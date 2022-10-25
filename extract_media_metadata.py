import ffmpeg
import sys
from pprint import pprint # for printing Python dictionaries in a human-readable way
import datetime

# read the audio/video file from the command line arguments
# media_file = sys.argv[1]
# uses ffprobe command to extract all possible metadata from the media file
pprint(ffmpeg.probe('/home/ldprpc15/Desktop/VIDEOREG/captured_1.avi')["streams"])

def video_with_audio(number, filename, video):
    title_video = '/home/ldprpc15/Desktop/VIDEOREG/' + str(datetime.now()) + number + '.mp4'
    input_audio = ffmpeg.input(filename)
    input_video = ffmpeg.input(video)
    ffmpeg.concat(input_video, input_audio, v=1, a=1).output(title_video).run()

video_with_audio()