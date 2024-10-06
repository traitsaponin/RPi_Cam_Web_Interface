from picamera2.outputs import FfmpegOutput

# Global variables to track recording state
recording_started = False
recording_thread = None
output_filename = None


def start_recording(cam):
    """
    Starts video recording. Creates the output file and starts encoder in
    the Picamera2 thread.

    Args:
        cam: CameraCoreModel instance.
    """
    print("Start video recording.")
    if cam.capturing_video:
        cam.print_to_logfile("Already capturing. Ignore")
        return
    cam.print_to_logfile("Capturing started")
    output_path = cam.make_filename(
        cam.config["video_output_path"]
    )  # Generate output file name
    cam.current_video_path = output_path  # Remember pathname.
    cam.video_encoder.output = FfmpegOutput(
        output_path
    )  # Set FfmpegOutput as output for video encoding to immediately get an MP4.

    # Note: May be a better idea to use FileOutput instead for .h264 (better FPS and efficiency).
    # Note 2: FfmpegOutput won't work with Pyinstaller, as it uses subprocess, which Pyinstaller can't handle.

    cam.picam2.start_encoder(
        cam.video_encoder, cam.video_encoder.output, name="main"
    )  # Start the video encoder
    cam.capturing_video = True  # Update flag to indicate video is being captured
    cam.set_status("video")  # Set camera status to 'video'


def stop_recording(cam):
    """
    Stops recording. Generates the thumbnail and resets any motion detection
    flags there may have been.

    Args:
        cam: CameraCoreModel instance.
    """
    print("Stop video recording.")
    if not cam.capturing_video:
        cam.print_to_logfile("Already stopped. Ignore")
        return
    cam.print_to_logfile("Capturing stopped")
    if cam.video_encoder.running:  # Stop the encoder if it's running
        cam.picam2.stop_encoder()
        cam.generate_thumbnail("v", cam.current_video_path)
        cam.current_video_path = None  # Reset current video pathname.

    cam.capturing_video = False  # Update flag to indicate video capture has stopped
    cam.reset_motion_state()  # Reset motion detection
    cam.set_status("ready")  # Set camera status back to 'ready'


def toggle_cam_record(cam, status):
    """
    Starts or stops video recording based on the status provided.

    Args:
        cam: CameraCoreModel instance.
        status (bool): If True, starts recording. If False, stops recording.
    """
    if status:  # Start video recording
        start_recording(cam)
    else:  # Stop video recording
        stop_recording(cam)
