import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
import re
import os


class GcodeVisualizer:
    def __init__(self, root):
        self.root = root
        self.root.title("G-code Drawing Visualizer")
        self.root.geometry("900x700")

        # Variables
        self.gcode_lines = []
        self.drawing_segments = []  # Store segments between M3 and M5
        self.path_points = []  # All points for drawing

        # Workspace boundaries (match with gcode_generator defaults)
        self.x_min = 2
        self.x_max = 18
        self.y_min = 24
        self.y_max = 35

        # Create UI
        self.create_widgets()

    def create_widgets(self):
        # Main layout with panes
        main_frame = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Left panel for controls
        left_frame = ttk.Frame(main_frame, width=250)
        main_frame.add(left_frame, weight=20)

        # Right panel for visualization
        right_frame = ttk.Frame(main_frame)
        main_frame.add(right_frame, weight=80)

        # File loading section
        file_frame = ttk.LabelFrame(left_frame, text="G-code File", padding=10)
        file_frame.pack(fill=tk.X, padx=5, pady=5)

        ttk.Button(file_frame, text="Load G-code File", command=self.load_gcode).pack(fill=tk.X, pady=5)
        self.file_label = ttk.Label(file_frame, text="No file loaded")
        self.file_label.pack(fill=tk.X)

        # Workspace configuration
        workspace_frame = ttk.LabelFrame(left_frame, text="Workspace", padding=10)
        workspace_frame.pack(fill=tk.X, padx=5, pady=5)

        # X-axis config
        ttk.Label(workspace_frame, text="X range:").grid(row=0, column=0, sticky="w", padx=5, pady=2)
        x_frame = ttk.Frame(workspace_frame)
        x_frame.grid(row=0, column=1, sticky="we", pady=2)

        self.x_min_var = tk.DoubleVar(value=self.x_min)
        self.x_max_var = tk.DoubleVar(value=self.x_max)
        ttk.Entry(x_frame, textvariable=self.x_min_var, width=6).pack(side=tk.LEFT, padx=2)
        ttk.Label(x_frame, text="to").pack(side=tk.LEFT, padx=5)
        ttk.Entry(x_frame, textvariable=self.x_max_var, width=6).pack(side=tk.LEFT, padx=2)

        # Y-axis config
        ttk.Label(workspace_frame, text="Y range:").grid(row=1, column=0, sticky="w", padx=5, pady=2)
        y_frame = ttk.Frame(workspace_frame)
        y_frame.grid(row=1, column=1, sticky="we", pady=2)

        self.y_min_var = tk.DoubleVar(value=self.y_min)
        self.y_max_var = tk.DoubleVar(value=self.y_max)
        ttk.Entry(y_frame, textvariable=self.y_min_var, width=6).pack(side=tk.LEFT, padx=2)
        ttk.Label(y_frame, text="to").pack(side=tk.LEFT, padx=5)
        ttk.Entry(y_frame, textvariable=self.y_max_var, width=6).pack(side=tk.LEFT, padx=2)

        # Apply workspace button
        ttk.Button(workspace_frame, text="Apply Workspace",
                   command=self.update_workspace).grid(row=2, column=0, columnspan=2,
                                                       sticky="we", padx=5, pady=5)

        # Display options
        options_frame = ttk.LabelFrame(left_frame, text="Display Options", padding=10)
        options_frame.pack(fill=tk.X, padx=5, pady=5)

        self.show_numbers_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(options_frame, text="Show segment numbers",
                        variable=self.show_numbers_var,
                        command=self.redraw_visualization).pack(anchor="w", pady=2)

        self.show_moves_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(options_frame, text="Show non-drawing moves",
                        variable=self.show_moves_var,
                        command=self.redraw_visualization).pack(anchor="w", pady=2)

        # Drawing stats frame
        stats_frame = ttk.LabelFrame(left_frame, text="Drawing Statistics", padding=10)
        stats_frame.pack(fill=tk.X, padx=5, pady=5, expand=True)

        self.lines_var = tk.StringVar(value="G-code lines: 0")
        ttk.Label(stats_frame, textvariable=self.lines_var).pack(anchor="w", pady=2)

        self.segments_var = tk.StringVar(value="Drawing segments: 0")
        ttk.Label(stats_frame, textvariable=self.segments_var).pack(anchor="w", pady=2)

        self.total_distance_var = tk.StringVar(value="Total drawing distance: 0 cm")
        ttk.Label(stats_frame, textvariable=self.total_distance_var).pack(anchor="w", pady=2)

        # Save image button
        ttk.Button(left_frame, text="Save Visualization as Image",
                   command=self.save_visualization).pack(fill=tk.X, padx=5, pady=10)

        # Create matplotlib figure for visualization on the right
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.canvas = FigureCanvasTkAgg(self.fig, master=right_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Initial plot setup
        self.setup_plot()

        # Status bar
        self.status_var = tk.StringVar(value="Ready")
        status_bar = ttk.Label(self.root, textvariable=self.status_var, relief=tk.SUNKEN, anchor=tk.W)
        status_bar.pack(side=tk.BOTTOM, fill=tk.X)

    def setup_plot(self):
        """Initialize the plot for visualization"""
        self.ax.clear()
        self.ax.set_xlabel('X (cm)')
        self.ax.set_ylabel('Y (cm)')
        self.ax.set_title('G-code Drawing Visualization')

        # Set workspace limits
        self.ax.set_xlim(self.x_min_var.get() - 2, self.x_max_var.get() + 2)
        self.ax.set_ylim(self.y_min_var.get() - 2, self.y_max_var.get() + 2)

        # Draw workspace boundaries
        x_min = self.x_min_var.get()
        x_max = self.x_max_var.get()
        y_min = self.y_min_var.get()
        y_max = self.y_max_var.get()

        self.ax.plot([x_min, x_max, x_max, x_min, x_min],
                     [y_min, y_min, y_max, y_max, y_min],
                     'k--', alpha=0.7, label='Workspace')

        # Add grid
        self.ax.grid(True, linestyle='--', alpha=0.6)

        # Draw workspace labels
        self.ax.text(x_min, y_min - 0.5, f"({x_min}, {y_min})", fontsize=8)
        self.ax.text(x_max, y_min - 0.5, f"({x_max}, {y_min})", fontsize=8)
        self.ax.text(x_min, y_max + 0.5, f"({x_min}, {y_max})", fontsize=8)
        self.ax.text(x_max, y_max + 0.5, f"({x_max}, {y_max})", fontsize=8)

        self.canvas.draw()

    def update_workspace(self):
        """Update workspace dimensions from input fields"""
        try:
            self.x_min = self.x_min_var.get()
            self.x_max = self.x_max_var.get()
            self.y_min = self.y_min_var.get()
            self.y_max = self.y_max_var.get()

            # Validate values
            if self.x_min >= self.x_max or self.y_min >= self.y_max:
                raise ValueError("Invalid workspace dimensions: min must be less than max")

            self.redraw_visualization()
            self.status_var.set(f"Workspace updated: X={self.x_min} to {self.x_max}, Y={self.y_min} to {self.y_max}")

        except Exception as e:
            messagebox.showerror("Error", f"Failed to update workspace: {str(e)}")

    def load_gcode(self):
        """Load G-code from a file and visualize drawing segments"""
        try:
            file_path = filedialog.askopenfilename(
                title="Load G-code File",
                filetypes=[
                    ("G-code files", "*.gcode *.nc *.gc"),
                    ("Text files", "*.txt"),
                    ("All files", "*.*")
                ]
            )

            if not file_path:
                return

            with open(file_path, 'r') as f:
                self.gcode_lines = [line.strip() for line in f.readlines()]

            # Update UI
            filename = os.path.basename(file_path)
            self.file_label.config(text=filename)
            self.lines_var.set(f"G-code lines: {len(self.gcode_lines)}")
            self.status_var.set(f"Loaded {len(self.gcode_lines)} lines from {filename}")

            # Parse G-code and visualize
            self.parse_gcode_for_drawing()

        except Exception as e:
            messagebox.showerror("Error", f"Failed to load G-code: {str(e)}")

    def parse_gcode_for_drawing(self):
        """Parse G-code to extract drawing segments (between M3 and M5)"""
        # Reset data
        self.path_points = []
        self.drawing_segments = []

        # Initialize variables
        x, y = 0, 0
        pen_down = False
        current_segment = []
        segment_idx = 0

        # Parse each line
        for line in self.gcode_lines:
            # Remove comments
            if ';' in line:
                line = line[:line.find(';')]

            # Skip empty lines
            if not line.strip():
                continue

            # Check for pen up/down commands
            if 'M3' in line:
                pen_down = True
                current_segment = [(x, y)]  # Start a new segment

            elif 'M5' in line:
                if pen_down and current_segment:
                    current_segment.append((x, y))  # Add the final point
                    self.drawing_segments.append({
                        'id': segment_idx,
                        'points': current_segment,
                        'length': self.calculate_segment_length(current_segment)
                    })
                    segment_idx += 1
                    current_segment = []
                pen_down = False

            # Parse G0/G1 movement commands
            if any(cmd in line for cmd in ['G0', 'G1']):
                # Extract X and Y coordinates if present
                x_match = re.search(r'X(-?\d+\.?\d*)', line)
                y_match = re.search(r'Y(-?\d+\.?\d*)', line)

                # Update position if coordinates are found
                if x_match:
                    x = float(x_match.group(1))
                if y_match:
                    y = float(y_match.group(1))

                # Add point to current segment if pen is down
                if pen_down and current_segment:
                    current_segment.append((x, y))

                # Add point to overall path for non-drawing visualization
                self.path_points.append((x, y, pen_down))

        # Calculate total drawing distance
        total_length = sum(segment['length'] for segment in self.drawing_segments)

        # Update statistics
        self.segments_var.set(f"Drawing segments: {len(self.drawing_segments)}")
        self.total_distance_var.set(f"Total drawing distance: {total_length:.2f} cm")

        # Draw visualization
        self.draw_visualization()

    def calculate_segment_length(self, points):
        """Calculate the length of a segment from its points"""
        if len(points) < 2:
            return 0

        total_length = 0
        for i in range(1, len(points)):
            x1, y1 = points[i - 1]
            x2, y2 = points[i]
            distance = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
            total_length += distance

        return total_length

    def draw_visualization(self):
        """Draw all segments on the plot"""
        # Reset the plot
        self.setup_plot()

        # Draw non-drawing moves if enabled
        if self.show_moves_var.get():
            move_x, move_y = [], []
            for i in range(1, len(self.path_points)):
                if not self.path_points[i][2] and not self.path_points[i - 1][2]:  # Both points not drawing
                    move_x.extend([self.path_points[i - 1][0], self.path_points[i][0]])
                    move_y.extend([self.path_points[i - 1][1], self.path_points[i][1]])

            if move_x:
                self.ax.plot(move_x, move_y, 'b--', linewidth=0.5, alpha=0.4, label='Movement')

        # Draw each drawing segment
        for segment in self.drawing_segments:
            points = segment['points']
            x_vals, y_vals = zip(*points)

            # Plot the drawing segment
            self.ax.plot(x_vals, y_vals, 'r-', linewidth=2.0)

            # Add segment number if enabled
            if self.show_numbers_var.get() and len(points) > 1:
                # Find midpoint for label
                mid_idx = len(points) // 2
                mx, my = points[mid_idx]
                self.ax.text(mx, my, f"{segment['id'] + 1}", fontsize=9, color='darkred',
                             bbox=dict(facecolor='white', alpha=0.7, edgecolor='none'))

        # Add legend entries
        self.ax.plot([], [], 'r-', linewidth=2.0, label='Drawing')
        if self.show_moves_var.get():
            self.ax.plot([], [], 'b--', linewidth=0.5, alpha=0.4, label='Movement')

        # Update legend
        self.ax.legend(loc='upper right')

        # Refresh canvas
        self.canvas.draw()

    def redraw_visualization(self):
        """Redraw the visualization with current settings"""
        if hasattr(self, 'drawing_segments') and self.drawing_segments:
            self.draw_visualization()

    def save_visualization(self):
        """Save the visualization as an image file"""
        if not hasattr(self, 'drawing_segments') or not self.drawing_segments:
            messagebox.showwarning("Warning", "No drawing to save!")
            return

        try:
            # Ask for file name
            file_path = filedialog.asksaveasfilename(
                title="Save Visualization",
                defaultextension=".png",
                filetypes=[
                    ("PNG Image", "*.png"),
                    ("JPEG Image", "*.jpg"),
                    ("PDF Document", "*.pdf"),
                    ("All files", "*.*")
                ]
            )

            if not file_path:
                return

            # Save figure
            self.fig.savefig(file_path, dpi=300, bbox_inches='tight')
            self.status_var.set(f"Visualization saved to {os.path.basename(file_path)}")
            messagebox.showinfo("Success", f"Visualization saved to {file_path}")

        except Exception as e:
            messagebox.showerror("Error", f"Failed to save visualization: {str(e)}")


# Run the application
if __name__ == "__main__":
    root = tk.Tk()
    app = GcodeVisualizer(root)
    root.mainloop()