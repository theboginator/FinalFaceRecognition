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
	"io"
	"log"
	"math"
	"os/exec"
	"time"

	"gobot.io/x/gobot"
	"gobot.io/x/gobot/platforms/dji/tello"
	"gocv.io/x/gocv"
)

const (
	frameSize = 960 * 720 * 3
	MAXWIDTH  = 960 //drone FOV width in pixels
	MAXHEIGHT = 720
	CURSORX   = MAXWIDTH / 2
	CURSORY   = MAXHEIGHT / 2
	TOOCLOSE  = 350 //Any detected object wider than this width should stop the drone
	TOOFAR    = 50  //Any detected object smaller than this width should be dropped from detection
)

var left int
var right int
var top int
var bottom int
var length int
var height int
var centerL int
var centerH int
var crosshair = image.Point{CURSORX, CURSORY}  //Mark to appear at center of camera
var tracker = image.Point{CURSORX, CURSORY}    //Mark to stand in as our tracking mark. substitute for movement.
var correction = image.Point{CURSORX, CURSORY} //This is our simulated movement icon. We'll start it in the center with the crosshair.
var oldCRC uint16
var lastNewFrameTime time.Time
var frameDiff time.Duration
var timeDiff int64
var newImage bool
var moveDrone = false //control the drone movement rate at half the sample rate

var lastUpdate time.Time
var firstErrorPass = true //On first pass, set lastUpdate to time.Now, then make false.
var lastErrorX float64 = 0
var lastErrorY float64 = 0
var changeX float64 = 0
var changeY float64 = 0
var derivativeX float64 = 0
var derivativeY float64 = 0
var lastIntegralX float64 = 0
var lastIntegralY float64 = 0
var KpX = 0.05
var KpY = 0.05
var KiX float64 = 0.00001
var KiY float64 = 0.00001
var KdX float64 = -2.0
var KdY float64 = -2.0
var drone = tello.NewDriver("8890")

func main() {

	//window := opencv.NewWindowDriver()
	window := gocv.NewWindow("Demo2")
	classifier := gocv.NewCascadeClassifier()
	classifier.Load("haarcascade_frontalface_default.xml")
	defer classifier.Close()
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
		newCRC := tello.CalculateCRC16(buf)
		if newCRC != oldCRC {
			oldCRC = newCRC
			current := time.Now()
			frameDiff = current.Sub(lastNewFrameTime)
			lastNewFrameTime = time.Now()
			//fmt.Println("New image! Time since last frame: ", frameDiff)
			newImage = true
		} else {
			//fmt.Println("Old image!")
			newImage = false
		}
		gocv.Circle(&img, crosshair, 8, colornames.Magenta, 3) //Show the drone crosshair
		if newImage {
			imageRectangles := classifier.DetectMultiScale(img)
			for _, rect := range imageRectangles {
				center, size := getCenter(rect)
				if size.X > TOOFAR {
					//fmt.Println("Object size: ", size.X, "x", size.Y)
					gocv.Rectangle(&img, rect, colornames.Cadetblue, 3)          //Box our detection
					gocv.Circle(&img, center, 5, colornames.Cadetblue, 3)        //Mark the center of the detection
					gocv.Line(&img, center, crosshair, colornames.Aquamarine, 3) //Draw a line from center to crosshair
					//HERE WE WANT TO MOVE OUR TRACKER TOWARDS THE DETECTION USING PID CONTROL ALGORITHM
					newCorrection, movement := getTargetOutput(center, correction) //Run our error checking algorithm to determine where to go
					correction = newCorrection
					if moveDrone {
						if size.X > TOOCLOSE {
							fmt.Println("Too close!")
							drone.Land()
						} else if size.X > TOOFAR {
							gocv.Circle(&img, newCorrection, 10, colornames.Midnightblue, 3) //Mark the simulated movement to the detection
							handleMovement(movement)                                         //Move the drone based on the error-corrected X and Y velocities
							//time.Sleep(200)
						} else {
							drone.Hover()
						}
					}
					moveDrone = !moveDrone
				} else {
					fmt.Println("Too far away")
				}
				//log.Println("found a face,", rect, "of size ", rect.Size(), "with center ", center, "at ", time.Now())
			}
		} else {
			fmt.Println("No new data")
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

func getCenter(rect image.Rectangle) (image.Point, image.Point) {
	cornerA := image.Point{rect.Max.X, rect.Max.Y}
	cornerB := image.Point{rect.Min.X, rect.Min.Y}
	centerX := cornerB.X + (rect.Dx() / 2)
	centerY := cornerA.Y - (rect.Dy() / 2)
	return image.Point{centerX, centerY}, image.Point{rect.Dx(), rect.Dy()}
}

func getTargetOutput(target image.Point, tracker image.Point) (image.Point, image.Point) {
	currentTime := time.Now()
	if firstErrorPass || currentTime.Sub(lastUpdate).Milliseconds() == 0 {
		timeDiff = 60
		firstErrorPass = false
	} else {
		timeDiff = currentTime.Sub(lastUpdate).Milliseconds()
	}
	//fmt.Println("Time diff: ", timeDiff)
	diffX := float64(target.X - tracker.X)
	diffY := float64(target.Y - tracker.Y)
	changeX = lastErrorX - diffX
	changeY = lastErrorY - diffY
	//fmt.Println("DetectedX:", target.X, "DetectedY:", target.Y, "trackerX:", tracker.X, "trackerY:", tracker.Y)
	//fmt.Println("last integral: ", lastIntegralX, "diff:", diffX, "time:", timeDiff )
	integralX := lastIntegralX + diffX*float64(timeDiff)
	integralY := lastIntegralY + diffY*float64(timeDiff)
	derivativeX = (diffX - lastErrorX) / float64(timeDiff)
	derivativeY = (diffY - lastErrorY) / float64(timeDiff)
	//fmt.Println("pre-scaled integral X:" ,integralX,"pre-scaled integral Y:", integralY)
	//fmt.Println("scaled integral X:", (KiX*integralX), "scaled integral Y:", (KiY*integralY))
	//fmt.Println("error-lasterrorX:", diffX-lastErrorX, "error-lasterrorY:", diffY-lastErrorY, "timediff:", timeDiff)
	//fmt.Println("Pre-scaled derivative X:", derivativeX, "pre-scaled derivative Y:", derivativeY)
	//fmt.Println("scaled derivative X:", (KdX*derivativeX), "scaled derivative Y:", (KdY*derivativeY))
	velX := (KpX * diffX) + (KiX * integralX) + (KdX * derivativeX) //use PID control to calculate x velocity
	velY := (KpY * diffY) + (KiY * integralY) + (KdY * derivativeY) //use PID control to calculate Y velocity
	newX := float64(tracker.X) + velX                               //Where to move our tracking dot to
	newY := float64(tracker.Y) + velY                               //Where to move our tracking dot to
	lastUpdate = currentTime
	lastErrorX = diffX //gocv.Circle(&img, correction, 10, colornames.Midnightblue, 3) //Mark the simulated movement to the detection
	lastErrorY = diffY
	lastIntegralX = integralX
	lastIntegralY = integralY
	//fmt.Println("diffX:", diffX, "diffY:", diffY, "velX:", velX, "velY:", velY)
	return image.Point{X: int(newX), Y: int(newY)}, image.Point{X: int(velX), Y: int(velY)}
}

func handleMovement(target image.Point) {
	if target.X > 0 { //If X > 0, then face is to left of drone, so we want to move left
		if target.X > 10 { //Limit how fast we'll try to move at once
			drone.Right(10)
			fmt.Println("moved Right 10")
		} else {
			xval := int(math.Abs(float64(target.X)))
			drone.Right(xval)
			fmt.Println("moved Right ", xval)
		}
	} else if target.X < 0 { //If x < 0, then face is to right of drone, want to move right. simply get absolute value of x to get speed to hand off to drone command.
		if target.X < -10 { //Limit how fast we'll try to move at once
			drone.Left(10)
			fmt.Println("moved Left 10")
		} else {
			xval := int(math.Abs(float64(target.X)))
			drone.Left(xval)
			fmt.Println("moved Left ", xval)
		}
	} else {
		drone.Hover()
		fmt.Println("hovering")
	}
	if target.Y > 0 {
		if target.Y > 10 { //Limit how fast we'll try to move at once
			//drone.Up(10)
			fmt.Println("Moved up 10")
		} else {
			yval := int(math.Abs(float64(target.Y)))
			//drone.Up(yval)
			fmt.Println("Moved up ", yval)
		}
	} else if target.Y < 0 {
		if target.Y < -10 { //Limit how fast we'll try to move at once
			//drone.Down(10)
			fmt.Println("moved down 10")
		} else {
			yval := int(math.Abs(float64(target.Y)))
			//drone.Down(yval)
			fmt.Println("moved down ", yval)
		}
	} else {
		//drone.Hover()
	}
	//ADD CODE: IF detection is to small, move forward a bit
	//else if detection size OK, HOVER
	//IF detection size exceeds threshold, LAND
}
