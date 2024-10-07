import os
import time
import threading
import signal

from picamera2 import Picamera2
from core.model import CameraCoreModel
from utilities.preview import generate_preview
from utilities.record import toggle_cam_record
from utilities.capture import capture_still_request
from utilities.motion_detect import motion_detection_thread, setup_motion_pipe


def on_sigint_sigterm(sig, frame):
    """
    Signal handler for SIGINT and SIGTERM.
    Sets process_running to False, allowing graceful shutdown.

    Args:
        sig: Signal number.
        frame: Current stack frame.
    """
    print("Received signal: ")
    print(sig)
    CameraCoreModel.process_running = False


# Register signal handlers for graceful shutdown
signal.signal(signal.SIGINT, on_sigint_sigterm)
signal.signal(signal.SIGTERM, on_sigint_sigterm)


def update_status_file(model):
    """
    Updates the status file with the current camera status.

    Args:
        model: CameraCoreModel instance containing the status and config.
    """
    model.set_status()
    current_status = model.current_status  # Get the current status from the model
    status_filepath = model.config["status_file"]  # Path to the status file
    status_dir = os.path.dirname(
        status_filepath
    )  # Get the directory of the status file

    # Create the status directory if it doesn't exist
    if not os.path.exists(status_dir):
        os.makedirs(status_dir)

    # Write the current status to the status file
    if current_status:
        status_file = open(status_filepath, "w")
        status_file.write(current_status)
        status_file.close()


def setup_fifo(path):
    """
    Sets up the FIFO named pipe for receiving commands.

    Args:
        path: String containing filepath of control file.
    Returns:
        True upon success, False upon failure.
    """
    # Get/make directory for the FIFO control pipe
    fifo_dir = os.path.dirname(path)
    if not os.path.exists(fifo_dir):
        os.makedirs(fifo_dir)
    # If the control file (FIFO) doesn't exist, create it
    if not os.path.exists(path):
        print("ALERT: Control file does not exist. Making new FIFO control file.")
        os.mkfifo(path, 0o6666)
    # Open the FIFO file in non-blocking read mode and flush any existing data
    CameraCoreModel.fifo_fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK, 0o666)
    try:
        os.read(CameraCoreModel.fifo_fd, CameraCoreModel.MAX_COMMAND_LEN)  # Flush pipe
    except BlockingIOError as e:
        print("ERROR: FIFO pipe busy. " + str(e))
        os.close(CameraCoreModel.fifo_fd)  # Close the FIFO pipe
        return False
    return True


def parse_incoming_commands():
    """
    Continuously checks for incoming commands from the FIFO pipe.
    Valid commands are added to the command queue.
    """
    while CameraCoreModel.process_running:
        # print("Checking for command...")
        incoming_cmd = None
        fifo_fd = (
            CameraCoreModel.fifo_fd
        )  # Access the file descriptor for the FIFO pipe
        if fifo_fd:
            # Read and validate incoming commands from the pipe
            incoming_cmd = read_pipe(fifo_fd)
        if incoming_cmd:
            # Add the valid command to the command queue
            CameraCoreModel.cmd_queue_lock.acquire()
            CameraCoreModel.command_queue.append(incoming_cmd)
            CameraCoreModel.cmd_queue_lock.release()
        time.sleep(CameraCoreModel.fifo_interval)  # Wait before checking the pipe again


def read_pipe(fd):
    """
    Reads data from the FIFO pipe and checks if it is a valid command.

    Args:
        fd: File descriptor of the FIFO pipe.

    Returns:
        Tuple of command and parameters if valid, otherwise False.
    """
    # Read the contents from the pipe and remove any trailing whitespace
    contents = os.read(fd, CameraCoreModel.MAX_COMMAND_LEN)
    contents_str = contents.decode().rstrip()
    cmd_code = contents_str[:2]  # Extract the command code (first 2 characters)
    cmd_param = contents_str[
        3:
    ]  # Extract the command parameters (after first 2 characters)

    # Check if the command is valid based on predefined valid commands
    if len(contents_str) > 0:
        if cmd_code in CameraCoreModel.VALID_COMMANDS:
            print("Valid command received: " + cmd_code)
            print("Command parameters: " + cmd_param)
            return (cmd_code, cmd_param)  # Return the command code and parameters
        else:
            print("Invalid command: " + contents_str)
    return False  # Return False for invalid commands


def execute_command(cmd_tuple, model, threads):
    """
    Executes the given command based on its code.

    Args:
        cmd_tuple: Tuple containing command code and parameters.
        current_request: Current camera capture request.
        model: CameraCoreModel instance.
    """
    cmd_code, cmd_param = cmd_tuple  # Unpack the command tuple

    if cmd_code == "ru":  # 'ru' stands for "run"
        if cmd_param.startswith("0"):
            print("Stopping camera, encoders and preview/motion threads...")
            model.current_status = "halted"
            # Stop preview and motion-detection threads
            for t in threads:
                t.join()
            # Make new threads to replace them, but don't start them until restart is called.
            preview_thread = threading.Thread(target=show_preview, args=(model,))
            md_thread = threading.Thread(target=motion_detection_thread, args=(model,))
            threads.clear()
            threads.append(preview_thread)
            threads.append(md_thread)
            model.stop_all()
        else:
            print("Restarting camera, encoders and preview/motion threads...")
            model.restart()
            model.set_status()
            for t in threads:
                if not t.is_alive():
                    t.start()

    elif model.current_status != "halted":
        if cmd_code == "im":  # 'im' stands for "image capture"
            capture_still_request(model)
        elif cmd_code == "ca":  # 'ca' stands for "camera action" (start/stop video)
            if cmd_param.startswith("1"):
                print("Starting video recording...")
                toggle_cam_record(model, True)
                if cmd_param[2:].isnumeric():
                    print(f"Record duration: {cmd_param[2:]}")
            else:
                print("Stopping video recording...")
                toggle_cam_record(model, False)
        elif cmd_code == "md":  # 'md' stands for "motion detection"
            if (cmd_param == "0") or not cmd_param:
                print("Stopping motion detection...")
                model.motion_detection = False
                model.print_to_logfile("Internal motion detection stopped")
            else:
                print("Starting motion detection...")
                model.motion_detection = True
                model.print_to_logfile("Internal motion detection started")
        elif cmd_code == "mx":
            # No implementation for Mode 1 yet.
            if cmd_param == "0":
                # Internal mode.
                model.config["motion_mode"] = "internal"
            elif cmd_param == "2":
                # Monitor mode.
                model.config["motion_mode"] = "monitor"
        else:
            print("Invalid command execution attempt.")
            model.print_to_logfile("Unrecognised pipe command")
    else:
        print("Camera status is halted. Cannot execute command.")
    update_status_file(model)  # Update status file after command execution


def show_preview(cam):
    """
    Method used for preview thread. Continuously creates the preview image.
    Running in its own thread minimizes disruption from still capture and
    other command execution.

    Args:
        cam: Camera instance used to generate preview.
    """
    while cam.current_status != "halted":  # CameraCoreModel.process_running:
        # Capture the current camera request
        current_request = cam.capture_request()
        # Generate a preview for the current frame
        generate_preview(cam, current_request)
        # Release the current request after processing
        current_request.release()


def start_background_process(config_filepath):
    """
    Main background process that sets up the camera and handles the command loop.

    Args:
        config_filepath: Path to the configuration file.
    """
    print("Starting RasPyCam main process...")
    all_cameras = (
        Picamera2.global_camera_info()
    )  # Get information about attached cameras

    # Check if any cameras are attached
    if not all_cameras:
        print("No attached cameras detected. Exiting program.")
        return

    # Set up the first detected camera
    first_cam = all_cameras[0]["Num"]
    cam = CameraCoreModel(first_cam, config_filepath[0] if config_filepath else None)

    # Setup FIFO for receiving commands
    if not setup_fifo(cam.config["control_file"]):
        cam.teardown()
        return

    # Setup motion pipe file
    setup_motion_pipe(cam.config["motion_pipe"])

    # Set the process to running
    CameraCoreModel.process_running = True

    # Start a thread to continuously parse incoming commands
    cmd_processing_thread = threading.Thread(target=parse_incoming_commands)
    cmd_processing_thread.start()

    # Start another thread just for the preview.
    preview_thread = threading.Thread(target=show_preview, args=(cam,))
    preview_thread.start()

    # Start the thread for motion detection.
    md_thread = threading.Thread(target=motion_detection_thread, args=(cam,))
    md_thread.start()

    threads = [preview_thread, md_thread]
    update_status_file(cam)

    # Execute commands off the queue as they come in.
    while CameraCoreModel.process_running:
        # Check if mutex lock can be acquired (i.e. FIFO thread is not writing to the command queue)
        # before popping from the command queue and attempting to execute. If lock can't be acquiring,
        # skip and check on the next loop cycle instead of blocking.
        # Without being non-blocking, anyone spamming the FIFO with commands will freeze/delay this thread.
        cmd_queue = CameraCoreModel.command_queue
        cmd_queue_lock = CameraCoreModel.cmd_queue_lock
        if (
            (cmd_queue)
            and (cmd_queue_lock.acquire(blocking=False))
            and (cam.current_status)
        ):
            next_cmd = CameraCoreModel.command_queue.pop(0)  # Get the next command
            cmd_queue_lock.release()
            execute_command(next_cmd, cam, threads)

        time.sleep(0.01)  # Small delay before next iteration

    cam.current_status = "halted"
    cmd_processing_thread.join()  # Wait for command processing thread to finish
    for t in threads:
        # Terminate preview and motion-detection threads.
        if t.is_alive():
            t.join()
    cam.teardown()  # Teardown the camera and stop it
    update_status_file(cam)  # Update the status file with halted status
    os.close(CameraCoreModel.fifo_fd)  # Close the FIFO pipe
