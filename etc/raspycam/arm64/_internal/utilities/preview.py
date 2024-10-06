import os


def generate_preview(cam, request):
    """
    Generate a preview image from the camera request and save it to the specified preview path.
    The preview is temporarily saved and then renamed to prevent flickering issues.
    """
    preview_path = cam.config["preview_path"]
    preview_width = cam.config["preview_size"][0]  # Preview width from config
    preview_height = cam.config["preview_size"][1]  # Preview height from config

    # Create the preview image using specified dimensions
    preview_img = request.make_image("main", preview_width, preview_height)

    # Temporarily save the preview image to avoid conflicts when updating the file
    temp_path = preview_path + ".part.jpg"

    # Save the preview image and related metadata
    cam.picam2.helpers.save(preview_img, request.get_metadata(), temp_path)

    # Rename the temporary file to the actual preview path (avoids preview flickering)
    os.rename(temp_path, preview_path)
