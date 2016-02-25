gst-launch-0.10 -v tcpclientsrc host=192.168.43.188 port=9991 ! jpegdec ! ffmpegcolorspace ! xvimagesink
