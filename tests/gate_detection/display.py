"""Contains functions needed to mark points on an image, display the image, or save it"""

import cv2 as cv


def display_img(filename: str, point_x: int, point_y: int) -> None:
    """
    Displays an image with a given filename with a red dot on the given point.

    Parameters
    ----------
    filename : str
        Path to the desired image to be displayed
    point_x : int
        Pixel x value of the point to be drawn
    point_y : int
        Pixel y value of the point to be drawn
    """
    img: cv.typing.MatLike = cv.imread(filename)
    img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    output_img: cv.typing.MatLike = cv.cvtColor(img, cv.COLOR_GRAY2BGR)
    cv.circle(output_img, (point_x, point_y), 2, (0, 0, 255), -1)
    cv.imshow("image", output_img)
    if cv.waitKey(0) & 0xFF == 27:
        cv.destroyAllWindows()
