import cv2
from PIL import Image


def capture_still_request(cam):
    """
    Captures and saves a still image from the camera's lores stream.

    Args:
        cam: CameraCoreModel instance.
        request: Camera capture request containing the image data.
    """
    print("Taking still image...")
    metadata = cam.picam2.capture_metadata()
    img_array = cam.picam2.capture_array("raw")  # Extract raw image data.
    image_path = cam.make_filename(
        cam.config["image_output_path"]
    )  # Generate output file name

    # Convert the lores YUV image to RGB for saving as JPG
    # NO LONGER USED.... But Bayer conversion also isn't done yet.

    # img_array = request.make_array(
    #    "lores"
    # )  # Extract the low-resolution image data from the request
    # lores_to_rgb = cv2.cvtColor(
    #    img_array, cv2.COLOR_YUV420p2BGR
    # )  # Convert YUV to RGB using OpenCV
    # converted_image = Image.fromarray(
    #    lores_to_rgb
    # )  # Create a PIL Image from the RGB array

    # Convert Bayer image to RGB for saving as JPG
    # NOT COMPLETE!!
    # This will get a JPG, but it's rubbish and obvously requires more
    # processing. But at least it represents some of the processing
    # power needed to do the full conversion for testing purposes.
    bayer_to_rgb = cv2.cvtColor(
        img_array, cv2.COLOR_BayerRG2BGR
    )  # Convert Bayer image to RGB using OpenCV.
    converted_image = Image.fromarray(
        bayer_to_rgb
    )  # Create a PIL Image from the RGB array

    # Save the converted image using camera helper functions
    cam.picam2.helpers.save(converted_image, metadata, image_path)

    # Save a thumbnail for this image.
    cam.generate_thumbnail("i", image_path)
