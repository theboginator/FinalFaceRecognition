/*
You must have ffmpeg and OpenCV installed in order to run this code. It will connect to the Tello
and then open a window using OpenCV showing the streaming video.
How to run
        go run examples/tello_opencv.go
this is based on merging two existing tutorials:
https://medium.com/@fonseka.live/detect-faces-using-golang-and-opencv-fbe7a48db055
and
https://gobot.io/documentation/examples/tello_opencv/
and of course the classifier
https://raw.githubusercontent.com/opencv/opencv/master/data/haarcascades/haarcascade_frontalface_default.xml
added updates to make windows and mac friendly
https://medium.com/tarkalabs/automating-dji-tello-drone-using-gobot-2b711bf42af6
*/

package main

import (
	"fmt"
	"golang.org/x/image/colornames"
	"image"
	"image/color"
	"io"
	"log"
	"os/exec"
	"time"
	"gobot.io/x/gobot"
	"gobot.io/x/gobot/platforms/dji/tello"
	"gocv.io/x/gocv"
)

const (
	frameSize = 960 * 720 * 3
)

var save []image.Point
var hull gocv.Mat
var defects gocv.Mat
var green = color.RGBA{0, 255, 0, 0}
var newImg gocv.Mat


func main() {
	drone := tello.NewDriver("8890")
	//window := opencv.NewWindowDriver()
	window := gocv.NewWindow("Demo2")
	classifier := gocv.NewCascadeClassifier()
	classifier.Load("haarcascade_frontalface_default.xml")
	defer classifier.Close()
	classifier2 := gocv.NewCascadeClassifier()
	classifier2.Load("fist.xml")
	defer classifier2.Close()
	classifier3 := gocv.NewCascadeClassifier()
	classifier3.Load("palm.xml")
	defer classifier3.Close()
	ffmpeg := exec.Command("ffmpeg", "-i", "pipe:0", "-pix_fmt", "bgr24", "-vcodec", "rawvideo",
		"-an", "-sn", "-s", "960x720", "-f", "rawvideo", "pipe:1")
	ffmpegIn, _ := ffmpeg.StdinPipe()
	ffmpegOut, _ := ffmpeg.StdoutPipe()
	work := func() {

		if err := ffmpeg.Start(); err != nil {
			fmt.Println(err)
			return
		}
		//count:=0
		go func() {

		}()

		drone.On(tello.ConnectedEvent, func(data interface{}) {
			fmt.Println("Connected")
			drone.StartVideo()
			drone.SetExposure(1)
			drone.SetVideoEncoderRate(4)

			gobot.Every(100*time.Millisecond, func() {
				drone.StartVideo()
			})
		})

		drone.On(tello.VideoFrameEvent, func(data interface{}) {
			pkt := data.([]byte)
			if _, err := ffmpegIn.Write(pkt); err != nil {
				fmt.Println(err)
			}
		})
	}

	robot := gobot.NewRobot("tello",
		[]gobot.Connection{},
		[]gobot.Device{drone},
		work,
	)

	robot.Start(false)
	drone.TakeOff()
	for {
		buf := make([]byte, frameSize)
		if _, err := io.ReadFull(ffmpegOut, buf); err != nil {
			fmt.Println(err)
			continue
		}

		img, err := gocv.NewMatFromBytes(720, 960, gocv.MatTypeCV8UC3, buf)
		if err != nil {
			log.Print(err)
			continue
		}
		if img.Empty() {
			continue
		}

		imageRectangles := classifier.DetectMultiScale(img)

		for _, rect := range imageRectangles {
			log.Println("found a face,", rect)
			gocv.Rectangle(&img, rect, colornames.Cadetblue, 3)
		}

		imageRectangles2 := classifier2.DetectMultiScale(img)

		for _, rect2 := range imageRectangles2 {
			log.Println("found a fist,", rect2)
			gocv.Rectangle(&img, rect2, colornames.Hotpink, 5)
			if rect2.Dx() > 400 {
				drone.FrontFlip()
			}
		}

		imageRectangles3 := classifier3.DetectMultiScale(img)

		for _, rect3 := range imageRectangles3 {
			log.Println("watching gesture", rect3)
			gocv.Rectangle(&img, rect3, colornames.Black, 5)
			drone.Halt()
		}




		window.IMShow(img)
		if window.WaitKey(1) >= 0 {
			break
		}
		//if count < 1000{
		//
		//}
		//window.WaitKey(1)
		//count +=1
	}
}
