import json
from picamera2 import Picamera2
from picamera2.encoders import H264Encoder, JpegEncoder
from picamera2.outputs import FileOutput
from datetime import datetime
import threading
import shutil
import os


class CameraCoreModel:
    """
    The CameraCoreModel represents the core camera functionality.
    It holds the camera instance, configuration settings, and controls the camera's status and encoders.
    """

    MAX_COMMAND_LEN = 256  # Maximum length of commands received from pipe
    FIFO_MAX = 10  # Maximum number of commands that can be queued at once
    VALID_COMMANDS = ["ca", "im", "md", "mx", "ru"]

    # ['tl','px','bo','tv','vi','an','as','at','ac',
    #             'ab','sh','co','br','sa','is','vs','rl','ec','em','wb',
    #             'ag','mm','ie','ce','ro','fl','ri','ss','qu','pv','bi','ru',
    #             'md','sc','rs','bu','mn','mt','mi','ms','mb','me','mc','mx',
    #             'mf','mz','vm','vp','wd','sy','um','cn','st','ls','qp','hp']

    process_running = False
    fifo_fd = None  # File descriptor for the FIFO pipe
    fifo_interval = 1.00  # Time interval for checking the FIFO pipe for new commands
    command_queue = []  # Queue of commands to be executed
    cmd_queue_lock = threading.Lock()  # Lock for synchronising access to command_queue

    def __init__(self, camera_index, config_path):
        """Initialises the camera and loads the configuration."""
        self.picam2 = Picamera2(camera_index)
        self.config = {
            "preview_size": (512, 288),
            "preview_path": "/tmp/preview/cam_preview.jpg",
            "image_output_path": "/tmp/media/im_%i_%Y%M%D_%h%m%s.jpg",
            "lapse_output_path": "/tmp/media/tl_%i_%t_%Y%M%D_%h%m%s.jpg",
            "video_output_path": "/tmp/media/vi_%v_%Y%M%D_%h%m%s.mp4",
            "media_path": "/tmp/media",
            "status_file": "/tmp/status_mjpeg.txt",
            "control_file": "/tmp/FIFO",
            "motion_pipe": "/tmp/motionFIFO",
            "video_width": 1920,
            "video_height": 1080,
            "video_bitrate": 17000000,  # Default video bitrate for encoding
            "motion_mode": "internal",  # Equivalent of the RaspiMJPEG's motion_external setting.
            "motion_threshold": 7.0,  # Mean-Square-Error. Default value per Picamera2's sample program.
            "motion_initframes": 0,  # How many frames to delay before starting any actual motion detection
            "motion_startframes": 3,  # How many frames of motion needed before flagging as motion detected
            "motion_stopframes": 50,  # How many frames w/o motion needed before unflagging for detected motion
            "autostart": True,  # Whether to start the Picamera2 instance when program launches, without waiting for 'ru'.
            "motion_detection": False,  # Whether to auto-start Motion Detection when program launches, no effect unless autostart is true.
            "user_config": "/tmp/uconfig",  # User configuration file used by RPi Cam Web Interface to overwrite defaults.
            "log_file": "/tmp/scheduleLog.txt",  # Filepath to record "print_to_log()" messages.
            "log_size": 5000,  # Set to 0 to not write to log file.
            "motion_logfile": "/tmp/motionLog.txt",  # Log file recording motion events during Monitor mode.
        }

        # Set up internal flags.
        self.current_status = (
            None  # Holds the current status string of the camera system
        )

        self.still_image_index = 0  # Next image file index, based on count of image files in output directory.
        self.video_file_index = 0  # Next video file index, based on count of video files in output directory.
        self.capturing_still = (
            False  # Flag for whether still image capture is in progress
        )
        self.capturing_video = False  # Flag for whether video recording is in progress

        self.motion_detection = False  # Flag for motion detection mode status

        self.timelapse_on = False  # Flag for timelapse mode
        self.detected_motion = False  # Flag for whether motion has been detected by MD.
        self.motion_still_count = (
            0  # Counter for number of consecutive frames with no motion.
        )
        self.motion_active_count = (
            0  # Counter for number of consecutive frames with active motion.
        )
        self.current_video_path = (
            None  # Stores filename of video currently being recorded.
        )

        self.read_config_file(
            config_path
        )  # Loads config from the provided config file path

        self.make_output_directories()

        # Set image/video file indexes based on detected thumbnail counts in the folder(s).
        self.make_filecounts()

        # Create and configure the camera for video capture
        # Note: Enabling raw stream seems to cut FPS down to 20fps when also using
        # FfmpegOutput to save as .mp4, without raw enabled it gets 30fps on both main and lores.
        # Can still get 30fps when using FileOutput to output unencoded .h264 with Raw enabled.
        video_config = self.picam2.create_video_configuration(
            main={
                "size": (self.config["video_width"], self.config["video_height"]),
                "format": "RGB888",
            },
            lores={"size": (320, 240), "format": "YUV420"},
            raw={"size": self.picam2.sensor_resolution, "format": "SBGGR10"},
        )

        # Can improve performance, but does it by changing resolutions to fit the nearest
        # number most easily processed by the camera/ISP (i.e. from 1924x1082 to 1920x1080).
        #   self.picam2.align_configuration(video_config)

        # These are supposed to change the framerate, but don't seem to do much....
        # self.picam2.controls.FrameDurationLimits = (33333, 33333, 33333)
        # self.picam2.video_configuration.controls.FrameDurationLimits = (33333, 33333, 33333)

        self.picam2.configure(video_config)

        print(self.picam2.camera_controls)
        print(self.picam2.camera_configuration())

        self.video_encoder = None  # Initialise video encoder as None
        self.setup_encoders()  # Sets up JPEG and H264 encoders for image and video encoding

        # Set initial status of the camera depending on autostart flag
        if self.config["autostart"]:
            self.picam2.start()
            # Set initial status of motion detection
            if self.config["motion_detection"]:
                self.motion_detection = True
        else:
            print("no autostart")

    def stop_all(self):
        """Stops the Picamera2 instance and any encoders currently running."""
        if self.video_encoder.running:
            self.picam2.stop_encoder(self.video_encoder)
        self.picam2.stop()
        self.reset_motion_state()
        self.capturing_video = False
        self.capturing_still = False
        self.motion_detection = False
        self.timelapse_on = False

    def reset_motion_state(self):
        """Resets the internal state flags for motion detection."""
        self.detected_motion = False
        self.motion_still_count = 0
        self.motion_active_count = 0

    def restart(self):
        """Restarts the Picamera2 instance."""
        self.picam2.stop()
        self.picam2.start()
        self.current_status = "ready"

    def teardown(self):
        """Stops and closes the camera when shutting down."""
        if self.video_encoder.running:
            self.picam2.stop_encoder(self.video_encoder)
        self.picam2.stop()
        self.picam2.close()
        # Remove any preview images there may be in the directory.
        preview_img = self.config["preview_path"]
        preview_part = preview_img + ".part.jpg"
        if os.path.exists(preview_img):
            os.remove(preview_img)
        if os.path.exists(preview_part):
            os.remove(preview_part)

    def make_output_directories(self):
        """
        Makes directories for status file, media folder, video, image and
        timelapse output files if they don't exist.
        """
        preview_path = os.path.dirname(self.config["preview_path"])
        im_path = os.path.dirname(self.config["image_output_path"])
        tl_path = os.path.dirname(self.config["lapse_output_path"])
        video_path = os.path.dirname(self.config["video_output_path"])
        media_path = os.path.dirname(self.config["media_path"])
        status_path = os.path.dirname(self.config["status_file"])
        paths = [preview_path, im_path, tl_path, video_path, media_path, status_path]
        for path in paths:
            if not os.path.exists(path):
                os.makedirs(path)

    def setup_encoders(self):
        """Sets up the JPEG and H264 encoders for the camera."""
        self.jpeg_encoder = JpegEncoder()  # JPEG encoder for still images
        self.jpeg_encoder.output = FileOutput()  # Output destination for JPEG images
        self.video_encoder = H264Encoder(
            bitrate=self.config["video_bitrate"], framerate=False
        )
        self.video_encoder.size = self.picam2.camera_config["main"]["size"]
        self.video_encoder.format = self.picam2.camera_config["main"]["format"]

    def read_config_file(self, config_path):
        """Reads the configuration file and loads it into the model."""
        if not config_path:
            return
        with open(config_path, "r") as cf_file:
            configs_from_file = json.load(cf_file)
        self.process_configs_from_file(configs_from_file)

    def process_configs_from_file(self, parsed_configs):
        """Processes the parsed configurations and applies them to the model.
        Updates model configuration values with values parsed from the config file
        """
        self.config.update(parsed_configs)

    def capture_request(self):
        """Wrapper for capturing a camera request."""
        return self.picam2.capture_request()

    def set_status(self, status=None):
        """
        Sets the current status of the camera.
        Logic for handling transitions between various camera statuses adapted
        from RaspiMJPEG's RaspiMUtils.c updateStatus() function.
        """
        if status:
            self.current_status = status  # Set the status if provided
            return

        if not self.picam2.started:
            self.current_status = "halted"
        elif self.capturing_video:
            if self.motion_detection:
                self.current_status = "video_motion"
            else:
                self.current_status = "video"
        elif self.capturing_still:
            self.current_status = "image"
        else:
            if self.motion_detection:
                self.current_status = "motion"
            else:
                self.current_status = "ready"

    def make_filename(self, name):
        """Generates a file name based on the given naming scheme."""
        current_dt = datetime.now()  # Get the current date and time
        # Format various components of the filename such as date, time, and indices
        year_2d = ("%04d" % current_dt.year)[2:]
        year_4d = "%04d" % current_dt.year
        month = "%02d" % current_dt.month
        day = "%02d" % current_dt.day
        hour = "%02d" % current_dt.hour
        minute = "%02d" % current_dt.minute
        seconds = "%02d" % current_dt.second
        millisecs = "%03d" % round(current_dt.microsecond / 1000)
        img_index = "%04d" % self.still_image_index
        vid_index = "%04d" % self.video_file_index

        name = name.replace("%v", vid_index)
        name = name.replace("%i", img_index)
        name = name.replace("%y", year_2d)
        name = name.replace("%Y", year_4d)
        name = name.replace("%M", month)
        name = name.replace("%D", day)
        name = name.replace("%h", hour)
        name = name.replace("%m", minute)
        name = name.replace("%s", seconds)
        name = name.replace("%u", millisecs)
        return name.replace("%%", "%")

    def make_filecounts(self):
        """Find the counts of all types of output files in their directory and
        updates the config dict with them.... in theory. RaspiMJPEG actually does this
        a somewhat boneheaded way by not actually looking at the files themselves,
        but instead their thumbnails and extracts the type/count from the filenames of
        those, by looking at the highest existing number for the type."""
        image_count = 0
        video_count = 0
        # Find all thumbnails.
        all_files = os.listdir(os.path.dirname(self.config["image_output_path"]))
        all_files.extend(os.listdir(os.path.dirname(self.config["video_output_path"])))
        all_files = set(all_files)

        for f in all_files:
            # Strip the extension off.
            filename = os.path.basename(f)
            file_without_ext, ext = os.path.splitext(filename)

            # Check if it's a thumbnail file
            if ext == ".th":
                if "im_" in file_without_ext:
                    try:
                        count = int(file_without_ext.split("_")[1])
                        image_count = max(image_count, count)
                    except (IndexError, ValueError):
                        continue
                elif "vi_" in file_without_ext:
                    try:
                        count = int(file_without_ext.split("_")[1])
                        video_count = max(video_count, count)
                    except (IndexError, ValueError):
                        continue

        # Set the indexes to one greater than the last existing count.
        # These will be used for the next thumbnails that will be generated.
        self.still_image_index = image_count + 1
        self.video_file_index = video_count + 1

    def generate_thumbnail(self, filetype, filepath):
        """Generates a thumbnail for a file of the given type and path.
        There are 3 types of files RaspiMJPEG differentiates between:
        Images ('i'), videos ('v') and timelapse sequences ('t'). The thumbnails
        are named slightly differently depending on which type it is.
        As with RaspiMJPEG, just copies the preview JPG file to use as thumbnail.
        """
        filename, _ = os.path.splitext(filepath)
        count = None
        if (filetype == "i") or (filetype == "t"):
            count = self.still_image_index
            self.still_image_index += 1
        elif filetype == "v":
            count = self.video_file_index
            self.video_file_index += 1
        # Make actual thumbnail.
        thumbnail_path = filename + "." + filetype + str(count) + ".th.jpg"
        shutil.copyfile(self.config["preview_path"], thumbnail_path)

    def print_to_logfile(self, message):
        """
        Writes message to the specified log file. If log size is 0, does not
        write anything. No current functionality for limiting lines to log_size.
        RPi Cam Interface uses the same file to write its Scheduler logs to and
        differentiates between them by using [] for its own message timestamps while
        RaspiMJPEG uses {} for its message timestamps.
        """
        if self.config["log_size"] == 0:
            return

        timestring = "{" + datetime.now().strftime("%Y/%m/%d %H:%M:%S") + "} "
        contents = timestring + message + "\n"
        log_fd = os.open(self.config["log_file"], os.O_RDWR | os.O_NONBLOCK, 0o777)
        log_file = os.fdopen(log_fd, "a")
        log_file.write(contents)
        log_file.close()
