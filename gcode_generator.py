import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import numpy as np
import cv2
import os
from PIL import Image, ImageTk
import threading
import time


class CropRectangle:
    def __init__(self, canvas, min_size=10):
        self.canvas = canvas
        self.min_size = min_size

        # Initial rectangle coordinates (will be adjusted when image is loaded)
        self.start_x = 0
        self.start_y = 0
        self.end_x = 0
        self.end_y = 0

        # Default size for workspace (X: -20 to 0, Y: 20 to 34)
        self.workspace_ratio = 20 / 14  # width / height ratio

        # Rectangle object and handlers
        self.rect_id = None
        self.handles = []
        self.active_handle = None

        # Drag state
        self.dragging = False
        self.drag_start_x = 0
        self.drag_start_y = 0

        # Bind events
        self.canvas.bind("<ButtonPress-1>", self.on_press)
        self.canvas.bind("<B1-Motion>", self.on_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_release)

    def create_rectangle(self, image_width, image_height):
        """Create initial crop rectangle based on image dimensions and workspace ratio"""
        # Calculate initial crop size
        if image_width / image_height > self.workspace_ratio:
            # Image is wider than workspace ratio
            crop_height = image_height * 0.8
            crop_width = crop_height * self.workspace_ratio
        else:
            # Image is taller than workspace ratio
            crop_width = image_width * 0.8
            crop_height = crop_width / self.workspace_ratio

        # Center the crop rectangle on the image
        self.start_x = (image_width - crop_width) / 2
        self.start_y = (image_height - crop_height) / 2
        self.end_x = self.start_x + crop_width
        self.end_y = self.start_y + crop_height

        # Create the rectangle
        self.draw_rectangle()

    def draw_rectangle(self):
        """Draw the crop rectangle and its handles"""
        # Delete old rectangle and handles
        if self.rect_id:
            self.canvas.delete(self.rect_id)
        for handle in self.handles:
            self.canvas.delete(handle)
        self.handles = []

        # Draw new rectangle
        self.rect_id = self.canvas.create_rectangle(
            self.start_x, self.start_y, self.end_x, self.end_y,
            outline="red", width=2, dash=(5, 5)
        )

        # Draw corner handles for resizing
        handle_size = 6
        handle_positions = [
            (self.start_x, self.start_y, "nw"),  # Top-left
            (self.end_x, self.start_y, "ne"),  # Top-right
            (self.end_x, self.end_y, "se"),  # Bottom-right
            (self.start_x, self.end_y, "sw")  # Bottom-left
        ]

        for x, y, cursor in handle_positions:
            handle = self.canvas.create_rectangle(
                x - handle_size, y - handle_size,
                x + handle_size, y + handle_size,
                fill="white", outline="red", width=1,
                tags=cursor
            )
            self.handles.append(handle)

        # Draw a label indicating workspace dimensions
        text_x = (self.start_x + self.end_x) / 2
        text_y = self.start_y - 10
        workspace_label = self.canvas.create_text(
            text_x, text_y,
            text="X:-20→0, Y:20→34 (cm)",
            fill="red", font=("Arial", 9, "bold"),
            tags="workspace_label"
        )
        self.handles.append(workspace_label)

    def on_press(self, event):
        """Handle mouse press events for drag or resize operations"""
        x, y = event.x, event.y

        # Check if clicking on a resize handle
        for handle in self.handles[:-1]:  # Skip text label
            coords = self.canvas.coords(handle)
            # FIX: Check if coords has enough elements before accessing them
            if len(coords) >= 4 and coords[0] <= x <= coords[2] and coords[1] <= y <= coords[3]:
                self.active_handle = self.canvas.gettags(handle)[0]
                self.drag_start_x = x
                self.drag_start_y = y
                return

        # Check if clicking inside the rectangle (for moving)
        if (self.start_x <= x <= self.end_x and
                self.start_y <= y <= self.end_y):
            self.dragging = True
            self.drag_start_x = x
            self.drag_start_y = y
            return

    def on_drag(self, event):
        """Handle mouse drag events"""
        if not self.dragging and not self.active_handle:
            return

        x, y = event.x, event.y
        dx = x - self.drag_start_x
        dy = y - self.drag_start_y

        if self.dragging:
            # Move the entire rectangle
            self.start_x += dx
            self.start_y += dy
            self.end_x += dx
            self.end_y += dy
        elif self.active_handle:
            # Resize based on active handle
            if "n" in self.active_handle:  # Top edge
                self.start_y = min(self.end_y - self.min_size, self.start_y + dy)
            if "s" in self.active_handle:  # Bottom edge
                self.end_y = max(self.start_y + self.min_size, self.end_y + dy)
            if "w" in self.active_handle:  # Left edge
                self.start_x = min(self.end_x - self.min_size, self.start_x + dx)
            if "e" in self.active_handle:  # Right edge
                self.end_x = max(self.start_x + self.min_size, self.end_x + dx)

        # Maintain aspect ratio while resizing (20:14 = 10:7)
        if self.active_handle:
            width = self.end_x - self.start_x
            height = self.end_y - self.start_y
            required_width = height * self.workspace_ratio
            required_height = width / self.workspace_ratio

            if "n" in self.active_handle or "s" in self.active_handle:
                # Adjusting height, set width based on ratio
                center_x = (self.start_x + self.end_x) / 2
                half_required_width = required_width / 2
                self.start_x = center_x - half_required_width
                self.end_x = center_x + half_required_width
            else:
                # Adjusting width, set height based on ratio
                center_y = (self.start_y + self.end_y) / 2
                half_required_height = required_height / 2
                self.start_y = center_y - half_required_height
                self.end_y = center_y + half_required_height

        # Update rectangle
        self.draw_rectangle()
        self.drag_start_x = x
        self.drag_start_y = y

    def on_release(self, event):
        """Handle mouse release events"""
        self.dragging = False
        self.active_handle = None

    def get_crop_coordinates(self):
        """Return crop coordinates as (x1, y1, x2, y2)"""
        return (int(self.start_x), int(self.start_y),
                int(self.end_x), int(self.end_y))


class ImageToGcodeApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Image to G-code Converter")
        self.root.geometry("900x700")

        # Variables
        self.image_path = None
        self.processed_image = None
        self.original_image = None
        self.displayed_image = None
        self.image_scale = 1.0
        self.gcode_lines = []
        self.processing = False
        self.crop_rectangle = None
        self.canvas_image_id = None

        # Create UI
        self.create_widgets()

    def create_widgets(self):
        # Main frame
        main_frame = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Control panel
        control_frame = ttk.Frame(main_frame)
        main_frame.add(control_frame, weight=30)

        # Display panel
        display_frame = ttk.Frame(main_frame)
        main_frame.add(display_frame, weight=70)

        # --- CONTROL PANEL ---
        # Image selection
        image_frame = ttk.LabelFrame(control_frame, text="Input Image", padding=10)
        image_frame.pack(fill=tk.X, padx=5, pady=5)

        ttk.Button(image_frame, text="Select Image", command=self.select_image).pack(fill=tk.X, pady=5)
        self.image_label = ttk.Label(image_frame, text="No image selected")
        self.image_label.pack(fill=tk.X)

        # Add crop control buttons
        crop_button_frame = ttk.Frame(image_frame)
        crop_button_frame.pack(fill=tk.X, pady=5)

        ttk.Button(crop_button_frame, text="Apply Crop", command=self.apply_crop).pack(side=tk.LEFT, fill=tk.X,
                                                                                       expand=True, padx=2)
        ttk.Button(crop_button_frame, text="Reset Crop", command=self.reset_crop).pack(side=tk.LEFT, fill=tk.X,
                                                                                       expand=True, padx=2)

        # Workspace limits
        workspace_frame = ttk.LabelFrame(control_frame, text="Workspace Limits", padding=10)
        workspace_frame.pack(fill=tk.X, padx=5, pady=5)

        ttk.Label(workspace_frame, text="X-axis:").grid(row=0, column=0, sticky="w", padx=5, pady=2)
        self.x_min_var = tk.DoubleVar(value=-20)
        self.x_max_var = tk.DoubleVar(value=0)
        ttk.Label(workspace_frame, text="From:").grid(row=0, column=1, sticky="e")
        ttk.Entry(workspace_frame, textvariable=self.x_min_var, width=6).grid(row=0, column=2, padx=2)
        ttk.Label(workspace_frame, text="to:").grid(row=0, column=3, sticky="e")
        ttk.Entry(workspace_frame, textvariable=self.x_max_var, width=6).grid(row=0, column=4, padx=2)

        ttk.Label(workspace_frame, text="Y-axis:").grid(row=1, column=0, sticky="w", padx=5, pady=2)
        self.y_min_var = tk.DoubleVar(value=20)
        self.y_max_var = tk.DoubleVar(value=34)
        ttk.Label(workspace_frame, text="From:").grid(row=1, column=1, sticky="e")
        ttk.Entry(workspace_frame, textvariable=self.y_min_var, width=6).grid(row=1, column=2, padx=2)
        ttk.Label(workspace_frame, text="to:").grid(row=1, column=3, sticky="e")
        ttk.Entry(workspace_frame, textvariable=self.y_max_var, width=6).grid(row=1, column=4, padx=2)

        # Reference frame option
        self.draw_frame_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(workspace_frame, text="Draw reference frame", variable=self.draw_frame_var).grid(row=2,
                                                                                                         column=0,
                                                                                                         columnspan=5,
                                                                                                         sticky="w",
                                                                                                         padx=5, pady=5)

        # Conversion parameters
        params_frame = ttk.LabelFrame(control_frame, text="Conversion Parameters", padding=10)
        params_frame.pack(fill=tk.X, padx=5, pady=5)

        ttk.Label(params_frame, text="Line spacing (mm):").grid(row=0, column=0, sticky="w", padx=5, pady=2)
        self.spacing_var = tk.DoubleVar(value=0.5)
        ttk.Entry(params_frame, textvariable=self.spacing_var, width=6).grid(row=0, column=1, sticky="w")

        ttk.Label(params_frame, text="Threshold (0-255):").grid(row=1, column=0, sticky="w", padx=5, pady=2)
        self.threshold_var = tk.IntVar(value=128)
        ttk.Scale(params_frame, from_=0, to=255, variable=self.threshold_var, orient="horizontal").grid(row=1, column=1,
                                                                                                        sticky="we")

        ttk.Label(params_frame, text="Image scale:").grid(row=2, column=0, sticky="w", padx=5, pady=2)
        self.scale_var = tk.DoubleVar(value=1.0)
        ttk.Scale(params_frame, from_=0.1, to=2.0, variable=self.scale_var, orient="horizontal").grid(row=2, column=1,
                                                                                                      sticky="we")

        # Speed control
        ttk.Label(params_frame, text="Drawing speed:").grid(row=3, column=0, sticky="w", padx=5, pady=2)
        self.speed_var = tk.IntVar(value=500)
        speed_scale = ttk.Scale(params_frame, from_=100, to=2000, variable=self.speed_var, orient="horizontal")
        speed_scale.grid(row=3, column=1, sticky="we")

        self.invert_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(params_frame, text="Invert colors", variable=self.invert_var).grid(row=4, column=0,
                                                                                           columnspan=2, sticky="w",
                                                                                           padx=5, pady=2)

        self.dithering_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(params_frame, text="Use dithering", variable=self.dithering_var).grid(row=5, column=0,
                                                                                              columnspan=2, sticky="w",
                                                                                              padx=5, pady=2)

        self.zigzag_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(params_frame, text="Use zigzag", variable=self.zigzag_var).grid(row=6, column=0, columnspan=2,
                                                                                        sticky="w", padx=5, pady=2)

        self.draw_black_var = tk.BooleanVar(value=True)
        ttk.Radiobutton(params_frame, text="Draw black object", variable=self.draw_black_var, value=True).grid(row=7,
                                                                                                               column=0,
                                                                                                               sticky="w",
                                                                                                               padx=5,
                                                                                                               pady=2)
        ttk.Radiobutton(params_frame, text="Draw white object", variable=self.draw_black_var, value=False).grid(row=7,
                                                                                                                column=1,
                                                                                                                sticky="w",
                                                                                                                padx=5,
                                                                                                                pady=2)

        # Processing buttons
        process_frame = ttk.Frame(control_frame)
        process_frame.pack(fill=tk.X, padx=5, pady=10)

        ttk.Button(process_frame, text="Preview", command=self.preview_processing).pack(side=tk.LEFT, padx=5,
                                                                                        expand=True, fill=tk.X)
        ttk.Button(process_frame, text="Generate G-code", command=self.generate_gcode).pack(side=tk.RIGHT, padx=5,
                                                                                            expand=True, fill=tk.X)

        # Draw reference frame button
        draw_frame_btn = ttk.Button(process_frame, text="Draw Reference Frame",
                                    command=self.create_reference_frame)
        draw_frame_btn.pack(side=tk.BOTTOM, fill=tk.X, pady=5)

        # G-code info
        gcode_info_frame = ttk.LabelFrame(control_frame, text="G-code Information", padding=10)
        gcode_info_frame.pack(fill=tk.X, padx=5, pady=5)

        self.line_count_var = tk.StringVar(value="Scan lines: 0")
        ttk.Label(gcode_info_frame, textvariable=self.line_count_var).pack(anchor="w")

        self.file_size_var = tk.StringVar(value="Drawing strokes: 0")
        ttk.Label(gcode_info_frame, textvariable=self.file_size_var).pack(anchor="w")

        # Status bar
        status_frame = ttk.Frame(control_frame)
        status_frame.pack(fill=tk.X, side=tk.BOTTOM, padx=5, pady=5)

        self.progress_var = tk.IntVar()
        self.progress = ttk.Progressbar(status_frame, variable=self.progress_var, maximum=100)
        self.progress.pack(fill=tk.X)

        self.status_var = tk.StringVar(value="Ready")
        ttk.Label(status_frame, textvariable=self.status_var).pack(anchor="w")

        # --- DISPLAY PANEL ---
        # Tabs
        self.tabs = ttk.Notebook(display_frame)
        self.tabs.pack(fill=tk.BOTH, expand=True)

        # Image tab
        self.image_tab = ttk.Frame(self.tabs)
        self.tabs.add(self.image_tab, text="Image")

        # Canvas and scrollbars for image
        self.canvas_frame = ttk.Frame(self.image_tab)
        self.canvas_frame.pack(fill=tk.BOTH, expand=True)

        self.canvas = tk.Canvas(self.canvas_frame, bg="#f0f0f0", scrollregion=(0, 0, 1000, 1000))

        # Scrollbars for canvas
        canvas_vsb = ttk.Scrollbar(self.canvas_frame, orient="vertical", command=self.canvas.yview)
        canvas_hsb = ttk.Scrollbar(self.canvas_frame, orient="horizontal", command=self.canvas.xview)
        self.canvas.configure(yscrollcommand=canvas_vsb.set, xscrollcommand=canvas_hsb.set)

        # Pack scrollbars and canvas
        canvas_vsb.pack(side="right", fill="y")
        canvas_hsb.pack(side="bottom", fill="x")
        self.canvas.pack(fill="both", expand=True)

        # G-code tab
        self.gcode_tab = ttk.Frame(self.tabs)
        self.tabs.add(self.gcode_tab, text="G-code")

        gcode_frame = ttk.Frame(self.gcode_tab)
        gcode_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        self.gcode_text = tk.Text(gcode_frame, wrap=tk.NONE)
        self.gcode_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        yscroll = ttk.Scrollbar(gcode_frame, orient=tk.VERTICAL, command=self.gcode_text.yview)
        yscroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.gcode_text.config(yscrollcommand=yscroll.set)

        xscroll = ttk.Scrollbar(self.gcode_tab, orient=tk.HORIZONTAL, command=self.gcode_text.xview)
        xscroll.pack(side=tk.BOTTOM, fill=tk.X)
        self.gcode_text.config(xscrollcommand=xscroll.set)

        # Save G-code button
        save_frame = ttk.Frame(self.gcode_tab)
        save_frame.pack(fill=tk.X, padx=5, pady=5)

        ttk.Button(save_frame, text="Save G-code", command=self.save_gcode).pack(side=tk.RIGHT)
        ttk.Button(save_frame, text="Save Reference Frame Only", command=self.save_reference_frame).pack(side=tk.RIGHT,
                                                                                                         padx=5)

    def select_image(self):
        """Select image from file system"""
        file_path = filedialog.askopenfilename(
            title="Select Image",
            filetypes=[
                ("Images", "*.jpg *.jpeg *.png *.bmp *.gif"),
                ("All files", "*.*")
            ]
        )

        if not file_path:
            return

        self.image_path = file_path
        self.image_label.config(text=os.path.basename(file_path))

        # Read and display image
        self.original_image = cv2.imread(file_path)
        self.displayed_image = self.original_image.copy()
        self.display_image(self.original_image)

        # Switch to image tab
        self.tabs.select(self.image_tab)

    def display_image(self, img, processed=False):
        """Display image on canvas with crop rectangle and workspace indicators"""
        if img is None:
            return

        # Convert from BGR to RGB
        if len(img.shape) == 3:
            img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        else:
            # If grayscale or binary, convert to RGB
            img_rgb = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)

        # Get canvas dimensions
        canvas_width = self.canvas.winfo_width()
        canvas_height = self.canvas.winfo_height()

        if canvas_width <= 1:  # Canvas not yet rendered
            canvas_width = 600
            canvas_height = 500

        # Calculate ratio to fit image within canvas
        img_height, img_width = img_rgb.shape[:2]
        self.image_scale = min(canvas_width / img_width, canvas_height / img_height)
        new_width = int(img_width * self.image_scale)
        new_height = int(img_height * self.image_scale)

        # If not a processed preview image, add workspace indicators
        if not processed and isinstance(img, np.ndarray):  # FIX: Check if img is actually a numpy array
            # Add workspace boundaries visual indicators
            overlay = img_rgb.copy()

            # Get workspace dimensions
            x_min = self.x_min_var.get()
            x_max = self.x_max_var.get()
            y_min = self.y_min_var.get()
            y_max = self.y_max_var.get()
            workspace_width = x_max - x_min
            workspace_height = y_max - y_min

            # Calculate workspace ratio and image ratio
            workspace_ratio = workspace_width / workspace_height
            img_ratio = img_width / img_height

            # Add a semi-transparent colored overlay to represent the workspace fit
            margin = 20  # pixels from edge
            overlay_color = (200, 230, 200)  # light green

            cv2.rectangle(overlay,
                          (margin, margin),
                          (img_width - margin, img_height - margin),
                          overlay_color, -1)  # filled rectangle

            alpha = 0.3  # transparency
            cv2.addWeighted(overlay, alpha, img_rgb, 1 - alpha, 0, img_rgb)

            # Add workspace ratio text
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.6
            text_color = (0, 100, 0)  # dark green
            text = f"Workspace ratio: {workspace_ratio:.2f} (width/height)"
            text_size = cv2.getTextSize(text, font, font_scale, 1)[0]
            text_x = img_width // 2 - text_size[0] // 2

            cv2.putText(img_rgb, text,
                        (text_x, margin - 10),
                        font, font_scale, text_color, 1, cv2.LINE_AA)

            if abs(workspace_ratio - img_ratio) > 0.05:  # 5% tolerance
                warning = "⚠️ Image ratio doesn't match workspace! Use crop to adjust."
                cv2.putText(img_rgb, warning,
                            (margin, img_height - margin + 20),
                            font, font_scale, (0, 0, 150), 1, cv2.LINE_AA)

        # Resize image
        img_resized = cv2.resize(img_rgb, (new_width, new_height))

        # Convert to Tkinter format
        self.tk_img = ImageTk.PhotoImage(image=Image.fromarray(img_resized))

        # Clear old canvas
        self.canvas.delete("all")

        # Update canvas scroll region
        self.canvas.config(scrollregion=(0, 0, new_width, new_height))

        # Display new image
        self.canvas_image_id = self.canvas.create_image(
            0, 0,
            image=self.tk_img,
            anchor=tk.NW
        )

        # Create or update crop rectangle
        if not processed:
            if self.crop_rectangle is None:
                self.crop_rectangle = CropRectangle(self.canvas)
                # Initialize with custom workspace ratio
                self.crop_rectangle.workspace_ratio = (self.x_max_var.get() - self.x_min_var.get()) / \
                                                      (self.y_max_var.get() - self.y_min_var.get())
            self.crop_rectangle.create_rectangle(new_width, new_height)

        if processed:
            self.processed_image = img

    def apply_crop(self):
        """Apply crop to the current image and fit to workspace dimensions"""
        if self.original_image is None or self.crop_rectangle is None:
            messagebox.showwarning("Warning", "No image loaded or crop area defined!")
            return

        # Get crop coordinates
        x1, y1, x2, y2 = self.crop_rectangle.get_crop_coordinates()

        # Convert from canvas coordinates to original image coordinates
        orig_x1 = int(x1 / self.image_scale)
        orig_y1 = int(y1 / self.image_scale)
        orig_x2 = int(x2 / self.image_scale)
        orig_y2 = int(y2 / self.image_scale)

        # Ensure coordinates are within image bounds
        img_height, img_width = self.original_image.shape[:2]
        orig_x1 = max(0, min(orig_x1, img_width - 1))
        orig_y1 = max(0, min(orig_y1, img_height - 1))
        orig_x2 = max(0, min(orig_x2, img_width))
        orig_y2 = max(0, min(orig_y2, img_height))

        # Crop the image
        self.displayed_image = self.original_image[orig_y1:orig_y2, orig_x1:orig_x2].copy()

        # Get workspace dimensions
        x_min = self.x_min_var.get()
        x_max = self.x_max_var.get()
        y_min = self.y_min_var.get()
        y_max = self.y_max_var.get()
        workspace_width = x_max - x_min  # Should be 20
        workspace_height = y_max - y_min  # Should be 14

        # Calculate the current aspect ratio
        crop_height, crop_width = self.displayed_image.shape[:2]
        current_ratio = crop_width / crop_height
        workspace_ratio = workspace_width / workspace_height

        # If aspect ratios don't match closely, adjust crop to match workspace ratio
        if abs(current_ratio - workspace_ratio) > 0.05:  # 5% tolerance
            # Determine if width or height needs adjustment
            if current_ratio > workspace_ratio:
                # Too wide, need to crop width
                new_width = int(crop_height * workspace_ratio)
                excess_width = crop_width - new_width
                start_x = excess_width // 2
                self.displayed_image = self.displayed_image[:, start_x:start_x + new_width]
                messagebox.showinfo("Auto-Fit", "Image width was adjusted to match workspace ratio")
            else:
                # Too tall, need to crop height
                new_height = int(crop_width / workspace_ratio)
                excess_height = crop_height - new_height
                start_y = excess_height // 2
                self.displayed_image = self.displayed_image[start_y:start_y + new_height, :]
                messagebox.showinfo("Auto-Fit", "Image height was adjusted to match workspace ratio")

        # Display the cropped image
        self.display_image(self.displayed_image)

        # Update crop info in status bar
        crop_height, crop_width = self.displayed_image.shape[:2]
        current_ratio = crop_width / crop_height
        self.status_var.set(
            f"Image cropped to {crop_width}×{crop_height} (ratio: {current_ratio:.2f}, workspace ratio: {workspace_ratio:.2f})")

    def reset_crop(self):
        """Reset to original image"""
        if self.original_image is None:
            return

        self.displayed_image = self.original_image.copy()
        self.display_image(self.displayed_image)
        self.status_var.set("Crop reset to original image")

    def preview_processing(self):
        """Preview image processing result with horizontal lines and workspace boundaries"""
        if self.image_path is None:
            messagebox.showwarning("Warning", "Please select an image first!")
            return

        try:
            # Get parameters
            threshold = self.threshold_var.get()
            invert = self.invert_var.get()
            use_dithering = self.dithering_var.get()
            line_spacing = self.spacing_var.get() / 10  # mm to cm

            # Get workspace dimensions
            custom_x_min = self.x_min_var.get()
            custom_x_max = self.x_max_var.get()
            custom_y_min = self.y_min_var.get()
            custom_y_max = self.y_max_var.get()
            workspace_width = custom_x_max - custom_x_min  # Should be 20
            workspace_height = custom_y_max - custom_y_min  # Should be 14

            # Use currently displayed (possibly cropped) image
            img = self.displayed_image
            if img is None:
                raise ValueError("No image to process")

            # Convert to grayscale
            img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # Apply dithering if needed
            if use_dithering:
                img_gray = self.apply_dithering(img_gray)

            # Invert colors if needed
            if invert:
                img_gray = 255 - img_gray

            # Apply threshold to create binary image
            _, img_binary = cv2.threshold(img_gray, threshold, 255, cv2.THRESH_BINARY)

            # Store the processed binary image
            self.processed_image = img_binary

            # Create new image to display horizontal lines
            height, width = img_binary.shape
            preview_img = np.ones((height, width, 3), dtype=np.uint8) * 255  # White image

            # Calculate scale to fit image within workspace
            scale_factor = self.scale_var.get()
            actual_scale = scale_factor * min(workspace_width / width, workspace_height / height) * 0.95

            # Calculate image center position in workspace
            offset_x = 0  # This will be relative position in the preview
            offset_y = 0

            # Calculate line spacing based on scale
            line_step = max(1, int(line_spacing / (scale_factor * 0.1)))  # Adjust scale

            # Define target pixel value based on drawing mode
            target_value = 0 if self.draw_black_var.get() else 255

            # Draw horizontal lines for the OBJECT (black or white pixels depending on setting)
            for y in range(0, height, line_step):
                if y >= height:
                    continue

                # Find segments to draw in this row
                segments = []
                start_x = None

                for x in range(width):
                    # Only draw for target pixels
                    if img_binary[y, x] == target_value:
                        if start_x is None:
                            start_x = x
                    elif start_x is not None:
                        segments.append((start_x, x - 1))
                        start_x = None

                # Process last segment
                if start_x is not None:
                    segments.append((start_x, width - 1))

                # Draw segments
                for start_x, end_x in segments:
                    # Use thicker blue lines for better visibility
                    cv2.line(preview_img, (start_x, y), (end_x, y), (0, 0, 255), 2)

            # Draw workspace boundaries on preview
            # We'll create a semi-transparent overlay to show the workspace area
            overlay = preview_img.copy()

            # Calculate pixels per cm based on the scaling
            # We're visualizing what portion of the image will fit within the workspace
            pix_per_cm = width / workspace_width

            # Calculate the image boundaries that correspond to the workspace
            left_border = 10
            right_border = width - 10
            top_border = 10
            bottom_border = height - 10

            # Draw rectangle representing workspace boundaries
            cv2.rectangle(overlay, (left_border, top_border), (right_border, bottom_border),
                          (0, 128, 0), 2)

            # Add a grid to indicate scale
            grid_spacing_cm = 5  # Grid every 5cm
            grid_spacing_px = int(grid_spacing_cm * pix_per_cm)

            # Draw grid lines
            for x in range(left_border, right_border, grid_spacing_px):
                cv2.line(overlay, (x, top_border), (x, bottom_border), (200, 200, 200), 1)

            for y in range(top_border, bottom_border, grid_spacing_px):
                cv2.line(overlay, (left_border, y), (right_border, y), (200, 200, 200), 1)

            # Add labels
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.5
            cv2.putText(overlay, f"X:{custom_x_min}", (left_border, top_border - 5),
                        font, font_scale, (0, 128, 0), 1, cv2.LINE_AA)
            cv2.putText(overlay, f"X:{custom_x_max}", (right_border - 40, top_border - 5),
                        font, font_scale, (0, 128, 0), 1, cv2.LINE_AA)
            cv2.putText(overlay, f"Y:{custom_y_min}", (left_border - 40, bottom_border),
                        font, font_scale, (0, 128, 0), 1, cv2.LINE_AA)
            cv2.putText(overlay, f"Y:{custom_y_max}", (left_border - 40, top_border + 15),
                        font, font_scale, (0, 128, 0), 1, cv2.LINE_AA)

            # Add workspace title
            cv2.putText(overlay, "WORKSPACE PREVIEW",
                        (left_border + 20, top_border - 25),
                        font, font_scale * 1.5, (0, 0, 255), 2, cv2.LINE_AA)

            # Blend the overlay with the original preview
            alpha = 0.7  # Transparency factor
            cv2.addWeighted(overlay, alpha, preview_img, 1 - alpha, 0, preview_img)

            # Add helper text about what is being shown
            instruction_text = "Preview shows raster scan lines and workspace fit"
            cv2.putText(preview_img, instruction_text,
                        (10, height - 10),
                        font, font_scale, (0, 0, 0), 1, cv2.LINE_AA)

            # Display result (no crop rectangle needed for preview)
            self.display_image(preview_img, processed=True)

            # Switch to image tab
            self.tabs.select(self.image_tab)

            num_lines = len([i for i in range(0, height, line_step)])
            self.status_var.set(f"Preview complete - {num_lines} scan lines")

        except Exception as e:
            messagebox.showerror("Error", f"Image processing error: {str(e)}")
            self.status_var.set(f"Error: {str(e)}")

    def apply_dithering(self, image):
        """Apply Floyd-Steinberg dithering effect"""
        # Convert image to float
        img_float = image.astype(np.float32) / 255.0
        height, width = img_float.shape

        for y in range(0, height - 1):
            for x in range(1, width - 1):
                old_pixel = img_float[y, x]
                new_pixel = 1.0 if old_pixel > 0.5 else 0.0
                img_float[y, x] = new_pixel

                error = old_pixel - new_pixel

                img_float[y, x + 1] += error * 7 / 16
                img_float[y + 1, x - 1] += error * 3 / 16
                img_float[y + 1, x] += error * 5 / 16
                img_float[y + 1, x + 1] += error * 1 / 16

        # Convert back to binary image
        return (img_float * 255).astype(np.uint8)

    def generate_gcode(self):
        """Generate G-code from processed image"""
        if self.image_path is None:
            messagebox.showwarning("Warning", "Please select an image first!")
            return

        # Process image if not already done
        if self.processed_image is None:
            self.preview_processing()

        # Start G-code generation in a separate thread
        self.status_var.set("Generating G-code...")
        self.progress_var.set(0)
        self.processing = True

        thread = threading.Thread(target=self._run_gcode_generation)
        thread.daemon = True
        thread.start()

    def _run_gcode_generation(self):
        """Run G-code generation process for the main object"""
        try:
            # Custom workspace boundaries
            custom_x_min = self.x_min_var.get()
            custom_x_max = self.x_max_var.get()
            custom_y_min = self.y_min_var.get()
            custom_y_max = self.y_max_var.get()

            # Get parameters
            line_spacing = self.spacing_var.get()
            scale_factor = self.scale_var.get()
            use_zigzag = self.zigzag_var.get()
            draw_frame = self.draw_frame_var.get()
            drawing_speed = self.speed_var.get()

            # Calculate custom workspace size
            workspace_width = custom_x_max - custom_x_min  # Should be 20
            workspace_height = custom_y_max - custom_y_min  # Should be 14

            # Check if processed image exists
            if self.processed_image is None:
                raise ValueError("No processed image data available. Please run preview first.")

            # Read processed image
            img_binary = self.processed_image
            height, width = img_binary.shape

            # Calculate scale to fit within custom workspace
            actual_scale = scale_factor * min(workspace_width / width, workspace_height / height) * 0.95

            # Calculate image center position to center in custom workspace
            offset_x = custom_x_min + (workspace_width - width * actual_scale) / 2
            offset_y = custom_y_min + (workspace_height - height * actual_scale) / 2

            # Convert line_spacing from mm to cm
            line_spacing_cm = line_spacing / 10

            # Define target pixel value based on drawing mode
            target_value = 0 if self.draw_black_var.get() else 255

            # Create G-code header
            gcode = []
            gcode.append("; G-code generated from image using raster technique")
            gcode.append("; Source image: " + os.path.basename(self.image_path))
            gcode.append(f"; Image size: {width}x{height} pixels")
            gcode.append(
                f"; Custom Workspace: X={custom_x_min} to {custom_x_max}, Y={custom_y_min} to {custom_y_max} (cm)")
            gcode.append(f"; Line spacing: {line_spacing} mm")
            gcode.append("G21 ; Set units to mm")
            gcode.append("G90 ; Absolute positioning")
            gcode.append(f"F{drawing_speed} ; Set feedrate")
            gcode.append("M5 ; Pen up")

            # Draw reference frame if requested
            if draw_frame:
                gcode.append("; Drawing custom workspace frame")
                # Draw border rectangle
                gcode.append(f"G0 X{custom_x_min:.2f} Y{custom_y_min:.2f} ; Move to bottom-left corner")
                gcode.append("M3 ; Pen down")
                gcode.append(f"G1 X{custom_x_max:.2f} Y{custom_y_min:.2f} ; Draw bottom line")
                gcode.append(f"G1 X{custom_x_max:.2f} Y{custom_y_max:.2f} ; Draw right line")
                gcode.append(f"G1 X{custom_x_min:.2f} Y{custom_y_max:.2f} ; Draw top line")
                gcode.append(f"G1 X{custom_x_min:.2f} Y{custom_y_min:.2f} ; Draw left line")
                gcode.append("M5 ; Pen up")

                # Draw centerlines
                center_x = (custom_x_min + custom_x_max) / 2
                center_y = (custom_y_min + custom_y_max) / 2

                # Draw X center line
                gcode.append(f"G0 X{center_x:.2f} Y{custom_y_min:.2f} ; Move to bottom center")
                gcode.append("M3 ; Pen down")
                gcode.append(f"G1 X{center_x:.2f} Y{custom_y_max:.2f} ; Draw vertical center line")
                gcode.append("M5 ; Pen up")

                # Draw Y center line
                gcode.append(f"G0 X{custom_x_min:.2f} Y{center_y:.2f} ; Move to left center")
                gcode.append("M3 ; Pen down")
                gcode.append(f"G1 X{custom_x_max:.2f} Y{center_y:.2f} ; Draw horizontal center line")
                gcode.append("M5 ; Pen up")

            # Move to starting position for image
            gcode.append(f"G0 X{offset_x:.2f} Y{offset_y:.2f} ; Move to starting position")

            # Counters
            line_count = 0
            scan_lines = 0

            # Scan image and create G-code
            zigzag = False  # Zigzag direction tracker
            progress_step = 100 / height

            for y in range(height):
                # Calculate actual Y position (in cm)
                real_y = offset_y + (height - y - 1) * actual_scale

                # Skip if outside custom workspace
                if real_y < custom_y_min or real_y > custom_y_max:
                    continue

                # Process only every line_spacing_cm
                if y % max(1, int(line_spacing_cm / actual_scale)) != 0:
                    continue

                scan_lines += 1

                # Find all pixels to draw in this row
                pixels_to_draw = []

                # Get pixel indices in row
                range_x = range(width)
                if zigzag:  # Reverse scan direction for zigzag
                    range_x = range(width - 1, -1, -1)

                # FIX: Store pixels with explicit type casting to avoid type issues
                for x in range_x:
                    # Only draw for target pixels
                    if img_binary[y, x] == target_value:
                        real_x = offset_x + x * actual_scale
                        if custom_x_min <= real_x <= custom_x_max:
                            # Store exactly two values with explicit type casting
                            pixels_to_draw.append((int(x), float(real_x)))

                # Skip if no pixels to draw
                if not pixels_to_draw:
                    continue

                # Build segments with explicit validation
                current_segment = []
                for pixel_data in pixels_to_draw:
                    # FIX: Only process if pixel_data has exactly 2 elements
                    if len(pixel_data) == 2:
                        x, real_x = pixel_data

                        if not current_segment:
                            # Start new segment
                            current_segment = [(x, real_x)]
                        elif abs(x - current_segment[-1][0]) <= 1:
                            # Continue current segment
                            current_segment.append((x, real_x))
                        else:
                            # End current segment and start a new one
                            # FIX: More validation to avoid index errors
                            if len(current_segment) > 0 and len(current_segment[0]) == 2:
                                start_data = current_segment[0]
                                end_data = current_segment[-1]

                                # FIX: Safe unpacking with explicit indexing
                                start_x = start_data[0]
                                real_start_x = start_data[1]
                                end_x = end_data[0]
                                real_end_x = end_data[1]

                                # Move to segment start
                                gcode.append(f"G0 F{drawing_speed * 1.5:.0f} X{real_start_x:.2f} Y{real_y:.2f}")
                                gcode.append("M3 ; Pen down")
                                line_count += 1

                                # Draw to segment end
                                gcode.append(f"G1 F{drawing_speed:.0f} X{real_end_x:.2f} Y{real_y:.2f}")
                                gcode.append("M5 ; Pen up")

                            # Start new segment
                            current_segment = [(x, real_x)]

                # Process final segment with validation
                # FIX: Added more validation to prevent IndexError
                if current_segment and len(current_segment) > 0:
                    # FIX: Safely access first and last elements
                    if len(current_segment[0]) == 2 and len(current_segment[-1]) == 2:
                        start_data = current_segment[0]
                        end_data = current_segment[-1]

                        # Use explicit indexing to avoid tuple unpacking issues
                        start_x = start_data[0]
                        real_start_x = start_data[1]
                        end_x = end_data[0]
                        real_end_x = end_data[1]

                        # Move to segment start
                        gcode.append(f"G0 F{drawing_speed * 1.5:.0f} X{real_start_x:.2f} Y{real_y:.2f}")
                        gcode.append("M3 ; Pen down")
                        line_count += 1

                        # Draw to segment end
                        gcode.append(f"G1 F{drawing_speed:.0f} X{real_end_x:.2f} Y{real_y:.2f}")
                        gcode.append("M5 ; Pen up")

                # Change direction for next scan if zigzag
                if use_zigzag:
                    zigzag = not zigzag

                # Update progress
                progress = min(100, int(y * progress_step))
                self.root.after(0, lambda p=progress: self.progress_var.set(p))

                # Check if process was canceled
                if not self.processing:
                    return

            # G-code footer - move to safe position in custom workspace
            center_x = (custom_x_min + custom_x_max) / 2
            gcode.append(
                f"G0 F{drawing_speed * 1.5:.0f} X{center_x:.2f} Y{custom_y_max - 2:.2f} ; Move to safe position")
            gcode.append("M5 ; Ensure pen is up")
            gcode.append("; End of G-code")

            # Update results
            self.gcode_lines = gcode
            self.root.after(0, self.update_gcode_display, gcode, line_count, scan_lines)

        except Exception as e:
            # Fix: Store the error message first to avoid lambda capture issues
            error_msg = str(e)
            self.root.after(0, lambda err=error_msg: messagebox.showerror("Error", f"G-code generation error: {err}"))
            self.root.after(0, lambda err=error_msg: self.status_var.set(f"Error: {err}"))
        finally:
            self.processing = False

    def update_gcode_display(self, gcode, line_count, scan_lines):
        """Update G-code display after successful generation"""
        # Display G-code
        self.gcode_text.delete(1.0, tk.END)
        self.gcode_text.insert(tk.END, '\n'.join(gcode))

        # Update info
        self.line_count_var.set(f"Scan lines: {scan_lines}")
        self.file_size_var.set(f"Drawing strokes: {line_count}")

        # Update progress and status
        self.progress_var.set(100)
        self.status_var.set("G-code generation complete")

        # Switch to G-code tab
        self.tabs.select(self.gcode_tab)

    def save_gcode(self):
        """Save G-code to file with proper ASCII encoding"""
        if not self.gcode_lines:
            messagebox.showwarning("Warning", "No G-code to save!")
            return

        file_path = filedialog.asksaveasfilename(
            title="Save G-code",
            defaultextension=".gcode",
            filetypes=[
                ("G-code files", "*.gcode"),
                ("Text files", "*.txt"),
                ("All files", "*.*")
            ]
        )

        if not file_path:
            return

        try:
            # Use ASCII encoding only
            with open(file_path, 'w', encoding='ascii', errors='ignore') as f:
                # Ensure only ASCII characters
                ascii_gcode = []
                for line in self.gcode_lines:
                    ascii_line = ''.join(c for c in line if ord(c) < 128)
                    ascii_gcode.append(ascii_line)
                f.write('\n'.join(ascii_gcode))

            self.status_var.set(f"G-code saved to {os.path.basename(file_path)}")
            messagebox.showinfo("Success", f"G-code saved to {file_path}")

        except Exception as e:
            messagebox.showerror("Error", f"Cannot save file: {str(e)}")

    def create_reference_frame(self):
        """Create and display reference frame G-code for the workspace"""
        # Get workspace dimensions
        x_min = self.x_min_var.get()
        x_max = self.x_max_var.get()
        y_min = self.y_min_var.get()
        y_max = self.y_max_var.get()
        drawing_speed = self.speed_var.get()

        # Create G-code for reference frame
        gcode = []
        gcode.append("; Reference Frame for SCARA Robot")
        gcode.append(f"; Workspace: X={x_min} to {x_max}, Y={y_min} to {y_max} cm")
        gcode.append("G21 ; Set units to mm")
        gcode.append("G90 ; Absolute positioning")
        gcode.append(f"F{drawing_speed} ; Set feedrate")
        gcode.append("M5 ; Pen up")

        # Draw border rectangle
        gcode.append(f"G0 X{x_min:.2f} Y{y_min:.2f} ; Move to bottom-left")
        gcode.append("M3 ; Pen down")
        gcode.append(f"G1 X{x_max:.2f} Y{y_min:.2f} ; Bottom side")
        gcode.append(f"G1 X{x_max:.2f} Y{y_max:.2f} ; Right side")
        gcode.append(f"G1 X{x_min:.2f} Y{y_max:.2f} ; Top side")
        gcode.append(f"G1 X{x_min:.2f} Y{y_min:.2f} ; Left side")
        gcode.append("M5 ; Pen up")

        # Draw center crosshairs
        center_x = (x_min + x_max) / 2
        center_y = (y_min + y_max) / 2

        # Horizontal centerline
        gcode.append(f"G0 X{x_min:.2f} Y{center_y:.2f} ; Move to center left")
        gcode.append("M3 ; Pen down")
        gcode.append(f"G1 X{x_max:.2f} Y{center_y:.2f} ; Draw horizontal centerline")
        gcode.append("M5 ; Pen up")

        # Vertical centerline
        gcode.append(f"G0 X{center_x:.2f} Y{y_min:.2f} ; Move to center bottom")
        gcode.append("M3 ; Pen down")
        gcode.append(f"G1 X{center_x:.2f} Y{y_max:.2f} ; Draw vertical centerline")
        gcode.append("M5 ; Pen up")

        # Draw diagonals
        gcode.append(f"G0 X{x_min:.2f} Y{y_min:.2f} ; Move to bottom-left")
        gcode.append("M3 ; Pen down")
        gcode.append(f"G1 X{x_max:.2f} Y{y_max:.2f} ; Draw diagonal line")
        gcode.append("M5 ; Pen up")

        gcode.append(f"G0 X{x_min:.2f} Y{y_max:.2f} ; Move to top-left")
        gcode.append("M3 ; Pen down")
        gcode.append(f"G1 X{x_max:.2f} Y{y_min:.2f} ; Draw other diagonal")
        gcode.append("M5 ; Pen up")

        # Scale markers - X axis
        tick_size = 0.5  # cm
        for x in range(int(x_min), int(x_max) + 1, 5):
            gcode.append(f"G0 X{x:.2f} Y{y_min:.2f} ; Move to tick position")
            gcode.append("M3 ; Pen down")
            gcode.append(f"G1 X{x:.2f} Y{y_min + tick_size:.2f} ; Draw tick")
            gcode.append("M5 ; Pen up")

        # Scale markers - Y axis
        for y in range(int(y_min), int(y_max) + 1, 5):
            gcode.append(f"G0 X{x_min:.2f} Y{y:.2f} ; Move to tick position")
            gcode.append("M3 ; Pen down")
            gcode.append(f"G1 X{x_min + tick_size:.2f} Y{y:.2f} ; Draw tick")
            gcode.append("M5 ; Pen up")

        # Return to safe position
        gcode.append(f"G0 X{center_x:.2f} Y{y_max - 2:.2f} ; Move to safe position")
        gcode.append("M5 ; Ensure pen is up")
        gcode.append("; End of reference frame G-code")

        # Display G-code in the text widget
        self.gcode_lines = gcode
        self.gcode_text.delete(1.0, tk.END)
        self.gcode_text.insert(tk.END, '\n'.join(gcode))

        # Update status and switch to G-code tab
        self.status_var.set("Reference frame G-code created")
        self.tabs.select(self.gcode_tab)

    def save_reference_frame(self):
        """Save only the reference frame G-code to file"""
        try:
            # Get parameters
            x_min = self.x_min_var.get()
            x_max = self.x_max_var.get()
            y_min = self.y_min_var.get()
            y_max = self.y_max_var.get()
            drawing_speed = self.speed_var.get()

            # Create reference frame G-code
            gcode = []
            gcode.append("; Reference frame G-code")
            gcode.append(f"; Workspace: X={x_min} to {x_max}, Y={y_min} to {y_max} (cm)")
            gcode.append("G21 ; Set units to mm")
            gcode.append("G90 ; Absolute positioning")
            gcode.append(f"F{drawing_speed} ; Set feedrate")
            gcode.append("M5 ; Pen up")

            # Draw border rectangle
            gcode.append(f"G0 F{drawing_speed * 1.5:.0f} X{x_min:.2f} Y{y_min:.2f} ; Move to bottom-left corner")
            gcode.append("M3 ; Pen down")
            gcode.append(f"G1 F{drawing_speed:.0f} X{x_max:.2f} Y{y_min:.2f} ; Draw bottom line")
            gcode.append(f"G1 X{x_max:.2f} Y{y_max:.2f} ; Draw right line")
            gcode.append(f"G1 X{x_min:.2f} Y{y_max:.2f} ; Draw top line")
            gcode.append(f"G1 X{x_min:.2f} Y{y_min:.2f} ; Draw left line")
            gcode.append("M5 ; Pen up")

            # Draw centerlines
            center_x = (x_min + x_max) / 2
            center_y = (y_min + y_max) / 2

            # Draw X center line
            gcode.append(f"G0 F{drawing_speed * 1.5:.0f} X{center_x:.2f} Y{y_min:.2f} ; Move to bottom center")
            gcode.append("M3 ; Pen down")
            gcode.append(f"G1 F{drawing_speed:.0f} X{center_x:.2f} Y{y_max:.2f} ; Draw vertical center line")
            gcode.append("M5 ; Pen up")

            # Draw Y center line
            gcode.append(f"G0 F{drawing_speed * 1.5:.0f} X{x_min:.2f} Y{center_y:.2f} ; Move to left center")
            gcode.append("M3 ; Pen down")
            gcode.append(f"G1 F{drawing_speed:.0f} X{x_max:.2f} Y{center_y:.2f} ; Draw horizontal center line")
            gcode.append("M5 ; Pen up")

            # Draw diagonals for easier alignment
            gcode.append(f"G0 F{drawing_speed * 1.5:.0f} X{x_min:.2f} Y{y_min:.2f} ; Move to bottom-left")
            gcode.append("M3 ; Pen down")
            gcode.append(f"G1 F{drawing_speed:.0f} X{x_max:.2f} Y{y_max:.2f} ; Draw diagonal line")
            gcode.append("M5 ; Pen up")

            gcode.append(f"G0 F{drawing_speed * 1.5:.0f} X{x_min:.2f} Y{y_max:.2f} ; Move to top-left")
            gcode.append("M3 ; Pen down")
            gcode.append(f"G1 F{drawing_speed:.0f} X{x_max:.2f} Y{y_min:.2f} ; Draw other diagonal")
            gcode.append("M5 ; Pen up")

            # Draw tick marks and labels
            tick_size = 0.5  # cm

            # X-axis ticks
            for x in range(int(x_min), int(x_max) + 1, 5):
                gcode.append(f"G0 F{drawing_speed * 1.5:.0f} X{x:.2f} Y{y_min:.2f} ; Move to tick position")
                gcode.append("M3 ; Pen down")
                gcode.append(f"G1 F{drawing_speed:.0f} X{x:.2f} Y{y_min + tick_size:.2f} ; Draw tick")
                gcode.append("M5 ; Pen up")

                # Y-axis ticks
                for y in range(int(y_min), int(y_max) + 1, 5):
                    gcode.append(f"G0 F{drawing_speed * 1.5:.0f} X{x_min:.2f} Y{y:.2f} ; Move to tick position")
                    gcode.append("M3 ; Pen down")
                    gcode.append(f"G1 F{drawing_speed:.0f} X{x_min + tick_size:.2f} Y{y:.2f} ; Draw tick")
                    gcode.append("M5 ; Pen up")

                    # Draw corner markers
                    marker_size = 1.0  # cm

                    # Bottom-left
                    gcode.append(f"G0 F{drawing_speed * 1.5:.0f} X{x_min:.2f} Y{y_min:.2f} ; Bottom-left corner")
                    gcode.append("M3 ; Pen down")
                    gcode.append(f"G1 F{drawing_speed:.0f} X{x_min + marker_size:.2f} Y{y_min:.2f}")
                    gcode.append(f"G1 X{x_min:.2f} Y{y_min:.2f}")
                    gcode.append(f"G1 X{x_min:.2f} Y{y_min + marker_size:.2f}")
                    gcode.append("M5 ; Pen up")

                    # Bottom-right
                    gcode.append(f"G0 F{drawing_speed * 1.5:.0f} X{x_max:.2f} Y{y_min:.2f} ; Bottom-right corner")
                    gcode.append("M3 ; Pen down")
                    gcode.append(f"G1 F{drawing_speed:.0f} X{x_max - marker_size:.2f} Y{y_min:.2f}")
                    gcode.append(f"G1 X{x_max:.2f} Y{y_min:.2f}")
                    gcode.append(f"G1 X{x_max:.2f} Y{y_min + marker_size:.2f}")
                    gcode.append("M5 ; Pen up")

                    # Top-right
                    gcode.append(f"G0 F{drawing_speed * 1.5:.0f} X{x_max:.2f} Y{y_max:.2f} ; Top-right corner")
                    gcode.append("M3 ; Pen down")
                    gcode.append(f"G1 F{drawing_speed:.0f} X{x_max - marker_size:.2f} Y{y_max:.2f}")
                    gcode.append(f"G1 X{x_max:.2f} Y{y_max:.2f}")
                    gcode.append(f"G1 X{x_max:.2f} Y{y_max - marker_size:.2f}")
                    gcode.append("M5 ; Pen up")

                    # Top-left
                    gcode.append(f"G0 F{drawing_speed * 1.5:.0f} X{x_min:.2f} Y{y_max:.2f} ; Top-left corner")
                    gcode.append("M3 ; Pen down")
                    gcode.append(f"G1 F{drawing_speed:.0f} X{x_min + marker_size:.2f} Y{y_max:.2f}")
                    gcode.append(f"G1 X{x_min:.2f} Y{y_max:.2f}")
                    gcode.append(f"G1 X{x_min:.2f} Y{y_max - marker_size:.2f}")
                    gcode.append("M5 ; Pen up")

                    # Return to safe position
                    gcode.append(
                        f"G0 F{drawing_speed * 1.5:.0f} X{center_x:.2f} Y{y_max - 2:.2f} ; Move to safe position")
                    gcode.append("M5 ; Ensure pen is up")
                    gcode.append("; End of reference frame G-code")

                    # Save to file
                    file_path = filedialog.asksaveasfilename(
                        title="Save Reference Frame G-code",
                        defaultextension=".gcode",
                        filetypes=[
                            ("G-code files", "*.gcode"),
                            ("Text files", "*.txt"),
                            ("All files", "*.*")
                        ]
                    )

                    if not file_path:
                        return

                    with open(file_path, 'w', encoding='ascii', errors='ignore') as f:
                        f.write('\n'.join(gcode))

                    self.status_var.set(f"Reference frame G-code saved to {os.path.basename(file_path)}")
                    messagebox.showinfo("Success", f"Reference frame G-code saved to {file_path}")

        except Exception as e:
            messagebox.showerror("Error", f"Cannot save reference frame: {str(e)}")

if __name__ == "__main__":
    root = tk.Tk()
    app = ImageToGcodeApp(root)
    root.mainloop()
