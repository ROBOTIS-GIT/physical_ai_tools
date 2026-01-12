# Image metadata for compressed MP4 storage
# This message is stored in ROSbag instead of full image data

std_msgs/Header header

# Frame index in MP4 video file
uint32 frame_index

# Original image dimensions
uint32 width
uint32 height

# Original encoding (e.g., "rgb8", "bgr8", "mono8")
string encoding

# Path to MP4 file (relative to bag directory)
string video_file_path

# Topic name that this frame came from
string source_topic
