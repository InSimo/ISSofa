#!/bin/bash
if [ "$*" == "" ]; then
echo "Usage: $0 output.avi fps step input_prefix [codec] [\"options\"]"
echo "Possible codecs are: ffv1 huffyuv mjpeg h264[default] llh264fast llh264slow"
exit 1
fi

PREFIX=${4:-seq}
FIRST=""
LAST=""

shopt -s nullglob
let i=0;
let n=${3:-1};
{
for f in ${PREFIX}*; do
#for f in capture*.png; do
    if [ $(($i%$n)) -eq 0 ]; then
        if [ -z "${FIRST}" ]; then
            FIRST=$f
        fi
        LAST=$f
        echo file $f
        echo duration 1
  fi
  let i+=1
done
echo file $TAIL
echo duration 30
echo file $TAIL
} > seqlist
echo Found $i frames
if [ $i -eq 0 ]; then
    exit 1
fi
#find . -name "seq*.bmp" > seqlist
#FIRST=`head -1 seqlist`
#TAIL=`tail -1 seqlist`
#RES=`identify -format %wx%h $FIRST`
#if [ -d seqframes ]; then rm -rf seqframes; fi
#mkdir seqframes
#let n=100000
#for f in `cat < seqlist`; do ln $f seqframes/frame-${n:1:5}.bmp || ln -s ../$f seqframes/frame-${n:1:5}.bmp; let n+=1; done

OPTS="-y -r ${2:-30}"
#OPTS="$OPTS -s $RES"
OPTS="$OPTS -f concat -i seqlist"

CODEC=${5:-h264}
case "$CODEC" in
ffv1)
  OPTS="$OPTS -pix_fmt yuv420p -flags +bitexact -vcodec ffv1"
  ;;
huffyuv)
  OPTS="$OPTS -pix_fmt yuv420p -flags +bitexact -vcodec huffyuv"
  ;;
mjpeg)
  OPTS="$OPTS -flags +bitexact -vcodec mjpeg -qmin 1 -qscale 1"
  ;;
h264)
#  OPTS="$OPTS -pix_fmt yuv420p -vcodec libx264 -cqp 0 -me_method dia -subq 1 -partitions -parti4x4-parti8x8-partp4x4-partp8x8-partb8x8"
#  OPTS="$OPTS -flags +bitexact+gray+umv+aic+aiv -vcodec libx264 -cqp 0"
#  OPTS="$OPTS -flags +bitexact+gray+umv+aic+aiv -vcodec libx264 -vpre lossless_medium -threads 0"
  OPTS="$OPTS -c:v libx264 -profile:v high -crf 20 -pix_fmt yuv420p"
  ;;
llh264fast)
  OPTS="$OPTS -flags +bitexact+gray+umv+aic+aiv -vcodec libx264 -vpre lossless_fast -threads 0"
  ;;
llh264slow)
  OPTS="$OPTS -flags +bitexact+gray+umv+aic+aiv -vcodec libx264 -vpre lossless_max -threads 0"
  ;;
*)
  echo "Unknown or unsupported codec $CODEC"
  exit 1
  ;;
esac



OPTS="$OPTS ${6} ${1:-video-lossless.avi}"

echo ffmpeg $OPTS
ffmpeg $OPTS
ls -l ${1:-video-lossless.avi}
