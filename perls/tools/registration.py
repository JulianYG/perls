import intera_interface
import argparse
import rospy

def main():
    """
    RSDK Head Display Example:
    Displays a given image file or multiple files on the robot's face.
    Pass the relative or absolute file path to an image file on your
    computer, and the example will read and convert the image using
    cv_bridge, sending it to the screen as a standard ROS Image Message.
    """
    epilog = """
	Notes:
   	Max screen resolution is 1024x600.
   	Images are always aligned to the top-left corner.
   	Image formats are those supported by OpenCv - LoadImage().
   	"""
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-f', '--file', nargs='+',
        help='Path to image file to send. Multiple files are separated by a space, eg.: a.png b.png'
    )
    parser.add_argument(
        '-l', '--loop', action="store_true",
        help='Display images in loop, add argument will display images in loop'
    )
    parser.add_argument(
        '-r', '--rate', type=float, default=1.0,
        help='Image display frequency for multiple and looped images'
    )
    args = parser.parse_args()

    rospy.init_node("head_display_example", anonymous=True)
 
    head_display = intera_interface.HeadDisplay()
    head_display.display_image(args.file, args.loop, args.rate)
 
if __name__ == '__main__':
    main()