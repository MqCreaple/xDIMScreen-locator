import numpy as np
import matplotlib.pyplot as plt

def draw_square(center: np.ndarray, size: float):
    """Draw a square centered at `center` with side length `size`"""
    half_size = size / 2
    pts=  np.array([
        center + np.array([half_size, half_size]),
        center + np.array([half_size, -half_size]),
        center + np.array([-half_size, -half_size]),
        center + np.array([-half_size, half_size]),
        center + np.array([half_size, half_size]),
    ])
    plt.plot(pts[:, 0], pts[:, 1], "b")


def left_90_degree(dir: np.ndarray) -> np.ndarray:
    """Rotate the 2D vector counterclockwise 90 degrees."""
    return np.array([-dir[1], dir[0]])

def draw_fractal(center: np.ndarray, size: float, square_type: int, dir: np.ndarray, lvl: int):
    """
    Iteratively draw the tag packing fractal.

    `center` is the center of the square. `size` is the side length of the square.

    `type` defines the current square's type. 0 means the central square; 1 means diagonal squares;
    2 means other squares.

    `dir` gives the direction of growth. For diagonal squares, `dir` is either (1, 0), (0, 1), (-1, 0),
    or (0, -1), representing the direction of the diagonal. For other squares, `dir` is either (1, 1),
    (1, -1), (-1, -1), or (-1, 1), representing the direction of the 2 sub-squares.
    """
    if lvl < 0:
        return
    draw_square(center, size)
    half_size = size / 2
    if square_type == 0:
        for dir in [np.array([1, 0]), np.array([0, 1]), np.array([-1, 0]), np.array([0, -1])]:
            draw_fractal(center + 4/3 * half_size * dir, size / 3, 1, dir, lvl - 1)
    elif square_type == 1:
        left = left_90_degree(dir)
        right = -left
        draw_fractal(
            center + left * size * 3/4 - dir * size / 4,
            half_size,
            2,
            dir + left,
            lvl - 1
        )
        draw_fractal(
            center + dir * 2/3 * size,
            size / 3,
            1,
            dir,
            lvl - 1
        )    # draw the smaller diagonal square
        draw_fractal(
            center + right * size * 3/4 - dir * size / 4,
            half_size,
            2,
            dir + right,
            lvl - 1
        )
    elif square_type == 2:
        draw_fractal(
            np.array([
                center[0] + dir[0] * size * 3/4,
                center[1] - dir[1] * size / 4,
            ]),
            half_size,
            2,
            dir,
            lvl - 1,
        )
        draw_fractal(
            np.array([
                center[0] - dir[0] * size / 4,
                center[1] + dir[1] * size * 3/4,
            ]),
            half_size,
            2,
            dir,
            lvl - 1,
        )

if __name__ == "__main__":
    fig = plt.figure()
    ax = fig.add_subplot(111)
    draw_fractal(np.array([0, 0]), 160, 0, np.array([0, 0]), 6)
    ax.set_aspect("equal", adjustable="box")
    plt.show()