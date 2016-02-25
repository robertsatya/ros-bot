GST="gst-launch-1.0 -v"
UDPCAPS="caps=\"application/x-rtp, media=video, clock-rate=90000, encoding-name=RAW, sampling=YCbCr-4:2:0, depth=(string)8, width=(string)320, height=(string)240, colorimetry=(string)BT601-5, payload=(int)96\""
SRC="udpsrc port=13375 $UDPCAPS"
DEC="rtpvrawdepay"
DST="xvimagesink"
PIPELINE="$SRC ! $DEC ! $DST"
$GST $PIPELINE
