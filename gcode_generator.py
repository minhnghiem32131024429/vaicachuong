import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import numpy as np
import cv2
import os
from PIL import Image, ImageTk
import threading
import time


class ImageToGcodeApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Image to G-code Converter")
        self.root.geometry("900x700")

        # Variables
        self.image_path = None
        self.processed_image = None
        self.original_image = None
        self.gcode_lines = []
        self.processing = False

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

        # Workspace limits
        workspace_frame = ttk.LabelFrame(control_frame, text="Workspace Limits", padding=10)
        workspace_frame.pack(fill=tk.X, padx=5, pady=5)

        ttk.Label(workspace_frame, text="X-axis:").grid(row=0, column=0, sticky="w", padx=5, pady=2)
        self.x_min_var = tk.DoubleVar(value=-20)
        self.x_max_var = tk.DoubleVar(value=10)
        ttk.Label(workspace_frame, text="From:").grid(row=0, column=1, sticky="e")
        ttk.Entry(workspace_frame, textvariable=self.x_min_var, width=6).grid(row=0, column=2, padx=2)
        ttk.Label(workspace_frame, text="to:").grid(row=0, column=3, sticky="e")
        ttk.Entry(workspace_frame, textvariable=self.x_max_var, width=6).grid(row=0, column=4, padx=2)

        ttk.Label(workspace_frame, text="Y-axis:").grid(row=1, column=0, sticky="w", padx=5, pady=2)
        self.y_min_var = tk.DoubleVar(value=20)
        self.y_max_var = tk.DoubleVar(value=38)
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

        self.canvas = tk.Canvas(self.image_tab, bg="#f0f0f0")
        self.canvas.pack(fill=tk.BOTH, expand=True)

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
        self.display_image(self.original_image)

    def display_image(self, img, processed=False):
        """Display image on canvas"""
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
        ratio = min(canvas_width / img_width, canvas_height / img_height)
        new_width = int(img_width * ratio)
        new_height = int(img_height * ratio)

        # Resize image
        img_resized = cv2.resize(img_rgb, (new_width, new_height))

        # Convert to Tkinter format
        self.tk_img = ImageTk.PhotoImage(image=Image.fromarray(img_resized))

        # Clear old canvas
        self.canvas.delete("all")

        # Display new image
        self.canvas.create_image(
            canvas_width // 2, canvas_height // 2,
            image=self.tk_img,
            anchor=tk.CENTER
        )

        # Draw workspace limits on canvas
        if self.draw_frame_var.get():
            # Calculate workspace limits in canvas coordinates
            x_min = self.x_min_var.get()
            x_max = self.x_max_var.get()
            y_min = self.y_min_var.get()
            y_max = self.y_max_var.get()

            # Display workspace dimensions on canvas
            self.canvas.create_text(
                canvas_width // 2, 20,
                text=f"Workspace: X: {x_min} to {x_max}, Y: {y_min} to {y_max} cm",
                fill="red", font=("Arial", 10)
            )

        if processed:
            self.processed_image = img

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

    def preview_processing(self):
        """Preview image processing result with horizontal lines"""
        if self.image_path is None:
            messagebox.showwarning("Warning", "Please select an image first!")
            return

        try:
            # Get parameters
            threshold = self.threshold_var.get()
            invert = self.invert_var.get()
            use_dithering = self.dithering_var.get()
            line_spacing = self.spacing_var.get() / 10  # mm to cm

            # Read image
            img = cv2.imread(self.image_path)
            if img is None:
                raise ValueError(f"Cannot read image from {self.image_path}")

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

            # Calculate line spacing based on scale
            scale_factor = self.scale_var.get()
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

            # Display result
            self.display_image(preview_img)

            # Switch to image tab
            self.tabs.select(self.image_tab)

            num_lines = len([i for i in range(0, height, line_step)])
            self.status_var.set(f"Preview complete - {num_lines} scan lines")

        except Exception as e:
            messagebox.showerror("Error", f"Image processing error: {str(e)}")
            self.status_var.set(f"Error: {str(e)}")

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
            # Get parameters
            x_min = self.x_min_var.get()
            x_max = self.x_max_var.get()
            y_min = self.y_min_var.get()
            y_max = self.y_max_var.get()
            line_spacing = self.spacing_var.get()
            scale_factor = self.scale_var.get()
            use_zigzag = self.zigzag_var.get()
            draw_frame = self.draw_frame_var.get()
            drawing_speed = self.speed_var.get()

            # Calculate workspace size
            workspace_width = x_max - x_min
            workspace_height = y_max - y_min

            # Read processed image
            img_binary = self.processed_image
            height, width = img_binary.shape

            # Calculate scale to fit
            actual_scale = scale_factor * min(workspace_width / width, workspace_height / height) * 0.95

            # Calculate image center position
            offset_x = x_min + (workspace_width - width * actual_scale) / 2
            offset_y = y_min + (workspace_height - height * actual_scale) / 2

            # Convert line_spacing from mm to cm
            line_spacing_cm = line_spacing / 10

            # Define target pixel value based on drawing mode
            target_value = 0 if self.draw_black_var.get() else 255

            # Create G-code header
            gcode = []
            gcode.append("; G-code generated from image using raster technique")
            gcode.append("; Source image: " + os.path.basename(self.image_path))
            gcode.append(f"; Image size: {width}x{height} pixels")
            gcode.append(f"; Workspace: X={x_min} to {x_max}, Y={y_min} to {y_max} (cm)")
            gcode.append(f"; Line spacing: {line_spacing} mm")
            gcode.append("G21 ; Set units to mm")
            gcode.append("G90 ; Absolute positioning")
            gcode.append(f"F{drawing_speed} ; Set feedrate")
            gcode.append("M5 ; Pen up")

            # Draw reference frame if requested
            if draw_frame:
                gcode.append("; Drawing reference frame")
                # Draw border rectangle
                gcode.append(f"G0 X{x_min:.2f} Y{y_min:.2f} ; Move to bottom-left corner")
                gcode.append("M3 ; Pen down")
                gcode.append(f"G1 X{x_max:.2f} Y{y_min:.2f} ; Draw bottom line")
                gcode.append(f"G1 X{x_max:.2f} Y{y_max:.2f} ; Draw right line")
                gcode.append(f"G1 X{x_min:.2f} Y{y_max:.2f} ; Draw top line")
                gcode.append(f"G1 X{x_min:.2f} Y{y_min:.2f} ; Draw left line")
                gcode.append("M5 ; Pen up")

                # Draw centerlines
                center_x = (x_min + x_max) / 2
                center_y = (y_min + y_max) / 2

                # Draw X center line
                gcode.append(f"G0 X{center_x:.2f} Y{y_min:.2f} ; Move to bottom center")
                gcode.append("M3 ; Pen down")
                gcode.append(f"G1 X{center_x:.2f} Y{y_max:.2f} ; Draw vertical center line")
                gcode.append("M5 ; Pen up")

                # Draw Y center line
                gcode.append(f"G0 X{x_min:.2f} Y{center_y:.2f} ; Move to left center")
                gcode.append("M3 ; Pen down")
                gcode.append(f"G1 X{x_max:.2f} Y{center_y:.2f} ; Draw horizontal center line")
                gcode.append("M5 ; Pen up")

                # Draw tick marks
                tick_size = 0.5  # cm

                # X-axis ticks
                for x in range(int(x_min), int(x_max) + 1, 5):
                    gcode.append(f"G0 X{x:.2f} Y{y_min:.2f} ; Move to tick position")
                    gcode.append("M3 ; Pen down")
                    gcode.append(f"G1 X{x:.2f} Y{y_min + tick_size:.2f} ; Draw tick")
                    gcode.append("M5 ; Pen up")

                # Y-axis ticks
                for y in range(int(y_min), int(y_max) + 1, 5):
                    gcode.append(f"G0 X{x_min:.2f} Y{y:.2f} ; Move to tick position")
                    gcode.append("M3 ; Pen down")
                    gcode.append(f"G1 X{x_min + tick_size:.2f} Y{y:.2f} ; Draw tick")
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

                # Skip if outside workspace
                if real_y < y_min or real_y > y_max:
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

                for x in range_x:
                    # Only draw for target pixels
                    if img_binary[y, x] == target_value:
                        real_x = offset_x + x * actual_scale
                        if x_min <= real_x <= x_max:
                            pixels_to_draw.append((x, real_x))

                # Skip if no pixels to draw
                if not pixels_to_draw:
                    continue

                # Build segments
                current_segment = []
                for i, (x, real_x) in enumerate(pixels_to_draw):
                    if not current_segment:
                        # Start new segment
                        current_segment = [(x, real_x)]
                    elif abs(x - current_segment[-1][0]) <= 1:
                        # Continue current segment
                        current_segment.append((x, real_x))
                    else:
                        # End current segment and start a new one
                        start_x, real_start_x = current_segment[0]
                        end_x, real_end_x = current_segment[-1]

                        # Move to segment start
                        gcode.append(f"G0 F{drawing_speed * 1.5:.0f} X{real_start_x:.2f} Y{real_y:.2f}")
                        gcode.append("M3 ; Pen down")
                        line_count += 1

                        # Draw to segment end
                        gcode.append(f"G1 F{drawing_speed:.0f} X{real_end_x:.2f} Y{real_y:.2f}")
                        gcode.append("M5 ; Pen up")

                        # Start new segment
                        current_segment = [(x, real_x)]

                # Process final segment
                if current_segment:
                    start_x, real_start_x = current_segment[0]
                    end_x, real_end_x = current_segment[-1]

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

            # G-code footer
            gcode.append(
                f"G0 F{drawing_speed * 1.5:.0f} X{x_min + (x_max - x_min) / 2:.2f} Y{y_max - 2:.2f} ; Move to safe position")
            gcode.append("M5 ; Ensure pen is up")
            gcode.append("; End of G-code")

            # Update results
            self.gcode_lines = gcode
            self.root.after(0, self.update_gcode_display, gcode, line_count, scan_lines)

        except Exception as e:
            self.root.after(0, lambda: messagebox.showerror("Error", f"G-code generation error: {str(e)}"))
            self.root.after(0, lambda: self.status_var.set(f"Error: {str(e)}"))
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
                f"G0 F{drawing_speed * 1.5:.0f} X{x_min + (x_max - x_min) / 2:.2f} Y{y_max - 2:.2f} ; Move to safe position")
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