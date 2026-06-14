import numpy as np
import rclpy
from matplotlib import pyplot as plt
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node


class GridValueVisualizer(Node):
    def __init__(self):
        super().__init__("grid_value_visualizer")

        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("center_row", -1)
        self.declare_parameter("center_col", -1)
        self.declare_parameter("window_size", 10)
        self.declare_parameter("show_all", True)
        self.declare_parameter("update_period", 2.0)
        self.declare_parameter("max_text_labels", 2500)

        self.last_draw_time = self.get_clock().now()
        self.last_map = None

        map_topic = self.get_parameter("map_topic").value
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            map_topic,
            self.map_callback,
            10,
        )

        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(9, 9))
        self.get_logger().info(f"Showing grid cell values from {map_topic}")

    def map_callback(self, msg):
        now = self.get_clock().now()
        update_period = self.get_parameter("update_period").value
        elapsed = (now - self.last_draw_time).nanoseconds / 1_000_000_000
        if elapsed < update_period:
            return

        width = msg.info.width
        height = msg.info.height

        if width == 0 or height == 0:
            self.get_logger().warn("Received an empty occupancy grid")
            return

        grid = np.array(msg.data, dtype=int).reshape((height, width))
        if self.last_map is not None and np.array_equal(grid, self.last_map):
            return

        self.last_draw_time = now
        self.last_map = grid.copy()

        show_all = self.get_parameter("show_all").value
        if show_all:
            r_start = 0
            r_end = height
            c_start = 0
            c_end = width
        else:
            center_row = self.get_parameter("center_row").value
            center_col = self.get_parameter("center_col").value
            window_size = self.get_parameter("window_size").value

            if center_row < 0:
                center_row = height // 2
            if center_col < 0:
                center_col = width // 2

            r_start = max(0, center_row - window_size)
            r_end = min(height, center_row + window_size + 1)
            c_start = max(0, center_col - window_size)
            c_end = min(width, center_col + window_size + 1)

        visible_grid = grid[r_start:r_end, c_start:c_end]
        self.draw_grid(visible_grid, r_start, c_start)

    def draw_grid(self, grid, row_offset, col_offset):
        self.ax.clear()
        self.ax.imshow(grid, cmap="gray_r", vmin=-1, vmax=100)

        rows, cols = grid.shape
        label_step = self.get_label_step(rows, cols)
        if label_step is not None:
            font_size = max(2, min(8, int(180 / max(rows, cols) * label_step)))
            for row in range(0, rows, label_step):
                for col in range(0, cols, label_step):
                    self.ax.text(
                        col,
                        row,
                        str(grid[row, col]),
                        ha="center",
                        va="center",
                        color="tab:red",
                        fontsize=font_size,
                    )

        self.ax.set_xticks(np.arange(-0.5, cols, label_step or 1), minor=True)
        self.ax.set_yticks(np.arange(-0.5, rows, label_step or 1), minor=True)
        self.ax.grid(which="minor", color="black", linewidth=0.5)
        self.ax.tick_params(which="minor", bottom=False, left=False)

        tick_step = max(1, max(rows, cols) // 20)
        x_ticks = range(0, cols, tick_step)
        y_ticks = range(0, rows, tick_step)
        self.ax.set_xticks(x_ticks)
        self.ax.set_yticks(y_ticks)
        self.ax.set_xticklabels(
            range(col_offset, col_offset + cols, tick_step),
            fontsize=7,
        )
        self.ax.set_yticklabels(
            range(row_offset, row_offset + rows, tick_step),
            fontsize=7,
        )
        self.ax.set_xlabel("column")
        self.ax.set_ylabel("row")

        if label_step is None:
            label_note = "no text labels"
        elif label_step == 1:
            label_note = "all cell labels"
        else:
            label_note = f"labels every {label_step} cells"
        self.ax.set_title(f"OccupancyGrid cell values ({label_note})")

        self.fig.tight_layout()
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def get_label_step(self, rows, cols):
        max_text_labels = self.get_parameter("max_text_labels").value
        if max_text_labels < 0:
            return None
        if max_text_labels == 0:
            return 1

        total_cells = rows * cols
        if total_cells <= max_text_labels:
            return 1

        return int(np.ceil(np.sqrt(total_cells / max_text_labels)))


def main(args=None):
    rclpy.init(args=args)
    node = GridValueVisualizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
