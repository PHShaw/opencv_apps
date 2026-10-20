# opencv_apps node reference

Every vision node in `opencv_apps`: what it does, the topics it uses, and its
parameters. Each parameter is also a launch argument of the same name in
`launch/<node>.launch.py`, and can be changed at runtime with
`ros2 param set /<node> <name> <value>` unless noted otherwise.

Topic names are shown as the node sees them. Input topics are relative
(`image`) and should be remapped to your camera, e.g.
`-r image:=/image_raw`. Output topics are node-private (`~/image`), so for a
node named `edge_detection` the result appears on `/edge_detection/image`.

- [Common parameters](#common-parameters)
- [Debug window sliders](#debug-window-sliders)
- [Filtering](#filtering): [smoothing](#smoothing), [morphology](#morphology), [pyramids](#pyramids), [threshold](#threshold), [equalize_histogram](#equalize_histogram), [discrete_fourier_transform](#discrete_fourier_transform)
- [Colour filtering](#colour-filtering): [rgb_color_filter](#rgb_color_filter), [hls_color_filter](#hls_color_filter), [hsv_color_filter](#hsv_color_filter), [lab_color_filter](#lab_color_filter)
- [Edges, features and shapes](#edges-features-and-shapes): [edge_detection](#edge_detection), [corner_harris](#corner_harris), [goodfeature_track](#goodfeature_track), [find_contours](#find_contours), [convex_hull](#convex_hull), [general_contours](#general_contours), [contour_moments](#contour_moments), [hough_lines](#hough_lines), [hough_circles](#hough_circles)
- [Optical flow](#optical-flow): [lk_flow](#lk_flow), [fback_flow](#fback_flow), [simple_flow](#simple_flow), [phase_corr](#phase_corr)
- [Tracking and segmentation](#tracking-and-segmentation): [camshift](#camshift), [segment_objects](#segment_objects), [watershed_segmentation](#watershed_segmentation)
- [Detection and recognition](#detection-and-recognition): [face_detection](#face_detection), [face_recognition](#face_recognition), [people_detect](#people_detect)
- [Combining images](#combining-images): [adding_images](#adding_images)
- [Examples](#examples): [simple_example](#simple_example), [simple_compressed_example](#simple_compressed_example)

## Common parameters

Every node except the two examples accepts these:

| Parameter | Type | Default | Description |
|---|---|---|---|
| `debug_view` | bool | `false` | Show an OpenCV window with the node's result. Most nodes also subscribe immediately when this is set. Needs a display. |
| `use_camera_info` | bool | `false` | Subscribe to `image` together with its `camera_info`, instead of `image` alone. |
| `queue_size` | int | `3` | Queue size of the input subscription (`1` for `discrete_fourier_transform`; `100` for `adding_images` and `face_recognition`, which pair up two inputs). |
| `always_subscribe` | bool | `false` | Subscribe to the input straight away. By default a node only subscribes once something subscribes to one of its outputs. |
| `verbose_connection` | bool | `false` | Log when the node subscribes or unsubscribes because of subscribers coming and going. |

The input image can be in any encoding cv_bridge can convert to the format the
node works in (for example `rgb8`, `bgr8`, `mono8` or `usb_cam`'s
`yuv422_yuy2`).

## Debug window sliders

With `debug_view:=true`, a node's window has a slider for each parameter
listed below. A slider and its parameter stay in sync both ways: dragging the
slider sets the parameter, and `ros2 param set` moves the slider. Each slider
is limited to its parameter's range.

- **On/off parameters** are 0/1 sliders.
- **Decimal parameters** move in fixed steps. Where the step is smaller than 1,
  the slider shows the value multiplied up and says so in its label. For
  example, `clahe_clip_limit x10` at 123 means 12.3. A value set with
  `ros2 param set` keeps its exact value, even between steps.
- **Most sliders are labelled with the parameter name.** Those added before
  this naming was adopted keep their older labels, given in brackets below.

| Node | Sliders |
|---|---|
| `adding_images` | `alpha`, `auto_beta`, `beta`, `gamma` |
| `camshift` | `vmin`, `vmax`, `smin` |
| `contour_moments` | `canny_low_threshold` ("Canny thresh:") |
| `convex_hull` | `threshold` ("Threshold:") |
| `corner_harris` | `threshold` ("Threshold:") |
| `edge_detection` | `edge_type` in every method's window; the Canny window adds `canny_threshold1`, `canny_threshold2`, `L2gradient`, `apply_blur_pre`, `apertureSize`, `apply_blur_post`, `postBlurSize`, `postBlurSigma` |
| `equalize_histogram` | `histogram_equalization_type`, `clahe_clip_limit`, `clahe_tile_size_x`, `clahe_tile_size_y` |
| `find_contours` | `canny_low_threshold` ("Canny thresh:") |
| `general_contours` | `threshold` ("Threshold:") |
| `goodfeature_track` | `max_corners` |
| `hough_circles` | every parameter (decimal ones move in whole steps) |
| `hough_lines` | `threshold`, `hough_type`, `rho`, `theta`, `minLineLength`, `maxLineGap` (decimal ones move in whole steps) |
| `lk_flow` | `min_distance`, `block_size`, `quality_level`, `harris_k` |
| `morphology` | `morph_operator`, `morph_element`, `morph_size` |
| `people_detect` | `hit_threshold`, `win_stride`, `padding`, `scale0`, `group_threshold` |
| `pyramids` | `pyramids_type`, `num_of_pyramids` |
| `rgb_color_filter`, `hls_color_filter`, `hsv_color_filter`, `lab_color_filter` | all six limits |
| `simple_flow` | `scale` |
| `smoothing` | `filter_type`, `kernel_size` |
| `threshold` | `threshold_type`, `threshold`, `max_binary`, `apply_otsu` |

## Filtering

### smoothing

Blurs the image to reduce noise.

- **Input:** `image`
- **Output:** `~/image` (`sensor_msgs/Image`, bgr8)

| Parameter | Type | Default | Range | Description |
|---|---|---|---|---|
| `filter_type` | int | `1` | 0–3 | 0 = homogeneous (box) blur, 1 = Gaussian blur, 2 = median blur, 3 = bilateral filter |
| `kernel_size` | int | `7` | 1–31 | Size of the square kernel. Rounded up to an odd number when set from the slider. |

### morphology

Applies a morphological operation, e.g. to remove speckles or fill small holes.

- **Input:** `image`
- **Output:** `~/image` (`sensor_msgs/Image`, bgr8)

| Parameter | Type | Default | Range | Description |
|---|---|---|---|---|
| `morph_operator` | int | `1` | 0–4 | 0 = opening, 1 = closing, 2 = gradient, 3 = top hat, 4 = black hat |
| `morph_element` | int | `0` | 0–2 | Kernel shape: 0 = rectangle, 1 = cross, 2 = ellipse |
| `morph_size` | int | `0` | 0–21 | Kernel radius. The kernel is `2 × morph_size + 1` pixels across. |

### pyramids

Scales the image up or down by factors of 2 using image pyramids.

- **Input:** `image`
- **Output:** `~/image` (`sensor_msgs/Image`, same encoding as the input, except that packed YUV and Bayer input is converted to bgr8)

| Parameter | Type | Default | Range | Description |
|---|---|---|---|---|
| `pyramids_type` | int | `0` | 0–1 | 0 = up (each level doubles the size), 1 = down (each level halves it) |
| `num_of_pyramids` | int | `1` | 1–10 | Number of levels to apply |

### threshold

Converts the image to grey and thresholds it.

- **Input:** `image`
- **Output:** `~/image` (`sensor_msgs/Image`, mono8)

| Parameter | Type | Default | Range | Description |
|---|---|---|---|---|
| `threshold_type` | int | `0` | 0–4 | 0 = binary, 1 = binary inverted, 2 = truncate, 3 = to zero, 4 = to zero inverted |
| `threshold` | int | `150` | 0–255 | Threshold value |
| `max_binary` | int | `255` | 0–255 | Value given to pixels that pass, for the binary types |
| `apply_otsu` | bool | `false` | | Choose the threshold automatically with Otsu's method (ignores `threshold`) |

### equalize_histogram

Converts the image to grey and spreads out its brightness levels to improve contrast.

- **Input:** `image`
- **Output:** `~/image` (`sensor_msgs/Image`, mono8)

| Parameter | Type | Default | Range | Description |
|---|---|---|---|---|
| `histogram_equalization_type` | int | `0` | 0–1 | 0 = global equalisation, 1 = CLAHE (equalises small tiles separately) |
| `clahe_clip_limit` | double | `20.0` | 0–255 | CLAHE contrast limit |
| `clahe_tile_size_x` | int | `32` | 0–300 | CLAHE tile width |
| `clahe_tile_size_y` | int | `32` | 0–300 | CLAHE tile height |
| `use_opencl` | bool | `true` | | Use OpenCV's OpenCL acceleration where available. Read at startup only. |

### discrete_fourier_transform

Converts the image to grey and publishes the magnitude of its Fourier
transform, with the zero frequency at the centre and a log scale.

- **Input:** `image`
- **Output:** `~/image` (`sensor_msgs/Image`, mono8)

No parameters beyond the [common ones](#common-parameters).

## Colour filtering

Each colour filter outputs a mask: white where a pixel's colour lies inside
all three ranges, black elsewhere. If a range's minimum is above its maximum,
the two are swapped, except for hue (see below).

- **Input:** `image`
- **Output:** `~/image` (`sensor_msgs/Image`, mono8 mask)

### rgb_color_filter

| Parameter | Type | Default | Range |
|---|---|---|---|
| `r_limit_min` / `r_limit_max` | int | `0` / `255` | 0–255 |
| `g_limit_min` / `g_limit_max` | int | `0` / `255` | 0–255 |
| `b_limit_min` / `b_limit_max` | int | `0` / `255` | 0–255 |

### hls_color_filter

| Parameter | Type | Default | Range |
|---|---|---|---|
| `h_limit_min` / `h_limit_max` | int | `0` / `360` | 0–360 (degrees) |
| `l_limit_min` / `l_limit_max` | int | `0` / `256` | 0–256 |
| `s_limit_min` / `s_limit_max` | int | `0` / `256` | 0–256 |

Hue wraps around: with `h_limit_min` above `h_limit_max`, e.g. 340 and 20,
the filter keeps hues from 340° up through 360°/0° to 20°, which suits reds.

### hsv_color_filter

| Parameter | Type | Default | Range |
|---|---|---|---|
| `h_limit_min` / `h_limit_max` | int | `0` / `360` | 0–360 (degrees) |
| `s_limit_min` / `s_limit_max` | int | `0` / `256` | 0–256 |
| `v_limit_min` / `v_limit_max` | int | `0` / `256` | 0–256 |

Hue wraps around in the same way as `hls_color_filter`.

### lab_color_filter

| Parameter | Type | Default | Range |
|---|---|---|---|
| `l_limit_min` / `l_limit_max` | int | `0` / `255` | 0–255 (L\*, lightness) |
| `a_limit_min` / `a_limit_max` | int | `0` / `255` | 0–255 (a\*, green to red) |
| `b_limit_min` / `b_limit_max` | int | `0` / `255` | 0–255 (b\*, blue to yellow) |

## Edges, features and shapes

### edge_detection

Finds edges with the Sobel, Laplace or Canny method.

- **Input:** `image`
- **Output:** `~/image` (`sensor_msgs/Image`, mono8 edge image)

| Parameter | Type | Default | Range | Description |
|---|---|---|---|---|
| `edge_type` | int | `0` | 0–2 | 0 = Sobel, 1 = Laplace, 2 = Canny |
| `canny_threshold1` | int | `100` | 0–500 | Canny: lower hysteresis threshold |
| `canny_threshold2` | int | `200` | 0–500 | Canny: upper hysteresis threshold |
| `apertureSize` | int | `3` | 1–10 | Aperture size of the Sobel operator |
| `L2gradient` | bool | `false` | | Canny: use the more accurate L2 norm for the gradient |
| `apply_blur_pre` | bool | `true` | | Blur the input before detecting edges |
| `apply_blur_post` | bool | `false` | | Gaussian-blur the edge image afterwards |
| `postBlurSize` | int | `13` | 3–31 | Kernel size of the post-blur |
| `postBlurSigma` | double | `3.2` | 0–10 | Sigma of the post-blur |

### corner_harris

Detects corners with the Harris detector and circles them on the image.

- **Input:** `image`
- **Output:** `~/image` (`sensor_msgs/Image`, bgr8)

| Parameter | Type | Default | Range | Description |
|---|---|---|---|---|
| `threshold` | int | `200` | 0–255 | Minimum normalised corner response to mark |

### goodfeature_track

Finds the strongest corners with the Shi–Tomasi method.

- **Input:** `image`
- **Outputs:** `~/image` (`sensor_msgs/Image`), `~/corners` (`opencv_apps/Point2DArrayStamped`)

| Parameter | Type | Default | Range | Description |
|---|---|---|---|---|
| `max_corners` | int | `23` | 1–100 | Maximum number of corners to return |

### find_contours

Finds Canny edges, then the contours (outlines) they form.

- **Input:** `image`
- **Outputs:** `~/image` (`sensor_msgs/Image`), `~/contours` (`opencv_apps/ContourArrayStamped`)

| Parameter | Type | Default | Range | Description |
|---|---|---|---|---|
| `canny_low_threshold` | int | `10` | 1–255 | Canny lower threshold (the upper is twice this) |

### convex_hull

Thresholds the image, finds contours and publishes the convex hull of each:
the smallest convex outline enclosing it.

- **Input:** `image`
- **Outputs:** `~/image` (`sensor_msgs/Image`), `~/hulls` (`opencv_apps/ContourArrayStamped`)

| Parameter | Type | Default | Range | Description |
|---|---|---|---|---|
| `threshold` | int | `100` | 1–255 | Threshold applied before finding contours |

### general_contours

Thresholds the image, finds contours and fits a rotated bounding rectangle and
an ellipse to each.

- **Input:** `image`
- **Outputs:** `~/image` (`sensor_msgs/Image`), `~/rectangles` and `~/ellipses` (`opencv_apps/RotatedRectArrayStamped`)

| Parameter | Type | Default | Range | Description |
|---|---|---|---|---|
| `threshold` | int | `100` | 1–255 | Threshold applied before finding contours |

### contour_moments

Finds Canny edges and contours, then publishes each contour's image moments,
centre, area and length, largest contour first. A contour that encloses no
area (an open line) gets the average of its points as its centre.

- **Input:** `image`
- **Outputs:** `~/image` (`sensor_msgs/Image`), `~/moments` (`opencv_apps/MomentArrayStamped`)

| Parameter | Type | Default | Range | Description |
|---|---|---|---|---|
| `canny_low_threshold` | int | `100` | 1–255 | Canny lower threshold (the upper is twice this) |

Per-contour details are logged at DEBUG level. Show them with
`--ros-args --log-level contour_moments:=debug`.

### hough_lines

Detects straight lines with the Hough transform. A colour input, or a grey
input that isn't already a binary edge image, is passed through Canny first.

- **Input:** `image`
- **Outputs:** `~/image` (`sensor_msgs/Image`), `~/lines` (`opencv_apps/LineArrayStamped`)

| Parameter | Type | Default | Range | Description |
|---|---|---|---|---|
| `hough_type` | int | `0` | 0–1 | 0 = standard transform, 1 = probabilistic transform (returns line segments) |
| `threshold` | int | `150` | 50–150 | Minimum votes for a line |
| `rho` | double | `1.0` | 1–100 | Distance resolution, in pixels |
| `theta` | double | `1.0` | 1–90 | Angle resolution, in degrees |
| `minLineLength` | double | `30.0` | 0–500 | Probabilistic only: shortest segment to keep |
| `maxLineGap` | double | `10.0` | 0–100 | Probabilistic only: largest gap to bridge within one segment |

### hough_circles

Detects circles with the Hough transform, after a Gaussian blur.

- **Input:** `image`
- **Outputs:** `~/image` (`sensor_msgs/Image`), `~/circles` (`opencv_apps/CircleArrayStamped`), `~/debug_image` (`sensor_msgs/Image`)

| Parameter | Type | Default | Range | Description |
|---|---|---|---|---|
| `canny_threshold` | double | `200.0` | 1–255 | Upper threshold of the internal Canny step |
| `accumulator_threshold` | double | `50.0` | 1–200 | Minimum votes for a circle. Lower finds more circles, including false ones. |
| `gaussian_blur_size` | int | `9` | 1–30 | Blur kernel size (should be odd) |
| `gaussian_sigma_x` | double | `2.0` | 1–10 | Blur sigma in x |
| `gaussian_sigma_y` | double | `2.0` | 1–10 | Blur sigma in y |
| `min_distance_between_circles` | double | `0.0` | 0–1024 | Minimum distance between circle centres |
| `dp` | double | `2.0` | 0–100 | Inverse ratio of the accumulator resolution to the image resolution |
| `min_circle_radius` | int | `0` | 0–500 | Smallest radius to find |
| `max_circle_radius` | int | `0` | 0–2000 | Largest radius to find (0 = no limit) |
| `debug_image_type` | int | `0` | 0–2 | What `~/debug_image` shows: 0 = input image, 1 = blurred image, 2 = Canny edges |

## Optical flow

The optical flow nodes compare consecutive frames to estimate how image
content moves.

### lk_flow

Tracks feature points from frame to frame with the Lucas–Kanade method.

- **Input:** `image`
- **Outputs:** `~/image` (`sensor_msgs/Image`), `~/flows` (`opencv_apps/FlowArrayStamped`)
- **Services** (`std_srvs/Empty`): `~/initialize_points` (find new points to track), `~/delete_points`, `~/toggle_night_mode` (draw on a black background)

| Parameter | Type | Default | Range | Description |
|---|---|---|---|---|
| `quality_level` | double | `0.01` | 0.001–10 | Minimum corner quality, relative to the best corner |
| `min_distance` | int | `10` | 1–100 | Minimum distance between tracked points |
| `block_size` | int | `3` | 1–100 | Neighbourhood size used when scoring corners |
| `harris_k` | double | `0.04` | 0.001–10 | Free parameter of the Harris detector |

Debug window keys: `r` auto-initialises the points, `c` deletes them all and
`n` toggles night mode.

### fback_flow

Computes dense optical flow (a motion vector at every pixel) with the
Farnebäck method.

- **Input:** `image`
- **Outputs:** `~/image` (`sensor_msgs/Image`), `~/flows` (`opencv_apps/FlowArrayStamped`)

No parameters beyond the [common ones](#common-parameters).

### simple_flow

Computes dense optical flow with the SimpleFlow method. It is slow: expect
well under one frame per second at full resolution.

- **Input:** `image`
- **Outputs:** `~/image` (`sensor_msgs/Image`), `~/flows` (`opencv_apps/FlowArrayStamped`)

| Parameter | Type | Default | Range | Description |
|---|---|---|---|---|
| `scale` | int | `4` | 1–24 | Shrink the input by this factor before computing the flow |

### phase_corr

Estimates how far the whole image has shifted since the previous frame, using
phase correlation.

- **Input:** `image`
- **Outputs:** `~/image` (`sensor_msgs/Image`), `~/shift` (`opencv_apps/Point2DStamped`)

No parameters beyond the [common ones](#common-parameters).

## Tracking and segmentation

### camshift

Tracks a coloured object with the CamShift algorithm, based on its hue
histogram. Start tracking by dragging a box round the object in the debug
window, or provide a histogram with the `histogram` parameter.

- **Input:** `image`
- **Outputs:** `~/image` (`sensor_msgs/Image`), `~/back_project` (`sensor_msgs/Image`, back-projection), `~/track_box` (`opencv_apps/RotatedRectStamped`)

| Parameter | Type | Default | Range | Description |
|---|---|---|---|---|
| `vmin` | int | `10` | 1–256 | Ignore pixels with brightness (V) below this |
| `vmax` | int | `256` | 1–256 | Ignore pixels with brightness (V) above this |
| `smin` | int | `30` | 1–256 | Ignore pixels with saturation below this |
| `histogram` | double array | `[]` | | Initial hue histogram (16 values). Read at startup only. |

Debug window keys: `c` stops tracking, `b` toggles the back-projection view,
`h` shows or hides the histogram, `p` pauses.

### segment_objects

Separates moving objects from the background using a learned background
model (MOG2). The output image shows the largest object filled in red.

- **Input:** `image`
- **Outputs:** `~/image` (`sensor_msgs/Image`), `~/contours` (`opencv_apps/ContourArrayStamped`, all objects), `~/area` (`std_msgs/Float64`, area of the largest object)
- **Service:** `~/update_bg_model` (`std_srvs/Empty`): toggle whether the background model keeps learning. The space key in the debug window does the same.

No parameters beyond the [common ones](#common-parameters).

### watershed_segmentation

Splits the image into regions with the watershed algorithm, starting from
seed marks. Draw rough marks on the debug window with the mouse (press `r` to
start again), or publish seed points.

- **Inputs:** `image`, `add_seed_points` (`opencv_apps/Point2DArray`)
- **Outputs:** `~/image` (`sensor_msgs/Image`), `~/contours` (`opencv_apps/ContourArrayStamped`)

No parameters beyond the [common ones](#common-parameters).

## Detection and recognition

### face_detection

Detects faces, and eyes within them, with Haar cascades.

- **Input:** `image`
- **Outputs:** `~/image` (`sensor_msgs/Image`), `~/face_image` (`sensor_msgs/Image`, crop of the largest face), `~/faces` (`opencv_apps/FaceArrayStamped`)

| Parameter | Type | Default | Description |
|---|---|---|---|
| `face_cascade_name` | string | `/usr/share/opencv4/haarcascades/haarcascade_frontalface_alt.xml` | Face cascade file. Read at startup only. |
| `eyes_cascade_name` | string | `/usr/share/opencv4/haarcascades/haarcascade_eye_tree_eyeglasses.xml` | Eye cascade file. Read at startup only. |

These are where Ubuntu's OpenCV 4 packages install the cascades. Set them if
your OpenCV is installed elsewhere.

### face_recognition

Recognises faces found by `face_detection` and labels them. Register faces
with the `~/train` service, or interactively with the trainer in
`face_recognition.launch.py`.

- **Inputs:** `image`, `faces` (`opencv_apps/FaceArrayStamped`, from `face_detection`)
- **Outputs:** `~/output` (`opencv_apps/FaceArrayStamped`, with labels and confidence), `~/debug_image` (`sensor_msgs/Image`)
- **Service:** `~/train` (`opencv_apps/FaceRecognitionTrain`)

| Parameter | Type | Default | Range | Description |
|---|---|---|---|---|
| `model_method` | string | `eigen` | | Recogniser: `eigen`, `fisher` or `LBPH` |
| `model_num_components` | int | `0` | 0–100 | Components kept by the eigen and fisher models (0 = automatic) |
| `model_threshold` | double | `8000.0` | 0–10000 | Distance above which a face counts as unknown |
| `lbph_radius` | int | `1` | 1–10 | LBPH only: radius |
| `lbph_neighbors` | int | `8` | 1–30 | LBPH only: number of neighbours |
| `lbph_grid_x` | int | `8` | 1–30 | LBPH only: grid cells across |
| `lbph_grid_y` | int | `8` | 1–30 | LBPH only: grid cells down |
| `face_model_width` | int | `190` | 30–500 | Width faces are resized to |
| `face_model_height` | int | `90` | 30–500 | Height faces are resized to |
| `face_padding` | double | `0.1` | 0–2 | Extra margin round each face, as a fraction of its size |
| `data_dir` | string | `~/.ros/opencv_apps/face_data` | | Where training images are saved |
| `use_saved_data` | bool | `true` | | Load saved training images at startup |
| `save_train_data` | bool | `true` | | Save new training images to `data_dir` |
| `approximate_sync` | bool | `false` | | Pair `image` and `faces` by approximate rather than exact timestamp |

This node has no `debug_view` or `use_camera_info` parameter.

### people_detect

Detects standing people with the HOG + linear SVM people detector.

- **Input:** `image`
- **Outputs:** `~/image` (`sensor_msgs/Image`), `~/found` (`opencv_apps/RectArrayStamped`)

| Parameter | Type | Default | Range | Description |
|---|---|---|---|---|
| `hit_threshold` | double | `0.0` | 0–10 | Detection threshold. Raise it to reduce false detections. |
| `win_stride` | int | `8` | 1–100 | Step of the detection window, in pixels |
| `padding` | int | `32` | 0–128 | Padding round the detection window |
| `scale0` | double | `1.05` | 1–100 | Scale step between detection passes |
| `group_threshold` | int | `2` | 0–100 | How many overlapping detections are needed to keep one (0 = keep all, no grouping) |

## Combining images

### adding_images

Blends two images: `alpha × image1 + beta × image2 + gamma`. The two
images are paired by timestamp, and should have the same size and encoding.

- **Inputs:** `image1`, `image2` (and `camera_info` with `use_camera_info:=true`)
- **Output:** `~/image` (`sensor_msgs/Image`)

| Parameter | Type | Default | Range | Description |
|---|---|---|---|---|
| `alpha` | double | `0.5` | 0–1 | Weight of `image1` |
| `beta` | double | `0.5` | 0–1 | Weight of `image2` (ignored when `auto_beta` is true) |
| `auto_beta` | bool | `true` | | Use `1 − alpha` as the weight of `image2` |
| `gamma` | double | `0.0` | 0–255 | Value added to every pixel |
| `approximate_sync` | bool | `true` | | Pair images by approximate rather than exact timestamp |

## Examples

Minimal nodes showing how to receive an image, process it with OpenCV and
publish the result. They don't use the common parameters or lazy
subscription.

### simple_example

Draws a red circle in the middle of each image.

- **Input:** `image`
- **Output:** `/image_converter/output_video/raw` (`sensor_msgs/Image`)
- **Parameters:** `debug_view` (bool, default `false`), `queue_size` (int, default `1`)

### simple_compressed_example

The same, using compressed images.

- **Input:** `image/compressed` (`sensor_msgs/CompressedImage`)
- **Output:** `/image_converter/output_video/compressed` (`sensor_msgs/CompressedImage`)
- **Parameters:** `debug_view` (bool, default `false`), `queue_size` (int, default `3`)
