namespace Microsoft.Samples.Kinect.BodyBasics
{
    using System;
    using System.Collections.Generic;
    using System.ComponentModel;
    using System.Diagnostics;
    using System.Globalization;
    using System.IO;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using Microsoft.Kinect;

    /// <summary>
    /// Interaction logic for MainWindow
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {
        int cnt = 0, i=0;
        StreamWriter file2 = new StreamWriter("D:\\kinect\\BodyBasics-WPF\\outputTemp.txt");
        bool gestureOn = false;
        List<List<double>> featureVector;
        /// <summary>
        /// Radius of drawn hand circles
        /// </summary>
        private const double HandSize = 30;

        /// <summary>
        /// Thickness of drawn joint lines
        /// </summary>
        private const double JointThickness = 3;

        /// <summary>
        /// Thickness of clip edge rectangles
        /// </summary>
        private const double ClipBoundsThickness = 10;

        /// <summary>
        /// Constant for clamping Z values of camera space points from being negative
        /// </summary>
        private const float InferredZPositionClamp = 0.1f;

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as closed
        /// </summary>
        private readonly Brush handClosedBrush = new SolidColorBrush(Color.FromArgb(128, 255, 0, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as opened
        /// </summary>
        private readonly Brush handOpenBrush = new SolidColorBrush(Color.FromArgb(128, 0, 255, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as in lasso (pointer) position
        /// </summary>
        private readonly Brush handLassoBrush = new SolidColorBrush(Color.FromArgb(128, 0, 0, 255));

        /// <summary>
        /// Brush used for drawing joints that are currently tracked
        /// </summary>
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));

        /// <summary>
        /// Brush used for drawing joints that are currently inferred
        /// </summary>        
        private readonly Brush inferredJointBrush = Brushes.Yellow;

        /// <summary>
        /// Pen used for drawing bones that are currently inferred
        /// </summary>        
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        /// <summary>
        /// Drawing group for body rendering output
        /// </summary>
        private DrawingGroup drawingGroup;

        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        private DrawingImage imageSource;

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;

        /// <summary>
        /// Coordinate mapper to map one type of point to another
        /// </summary>
        private CoordinateMapper coordinateMapper = null;

        /// <summary>
        /// Reader for body frames
        /// </summary>
        private BodyFrameReader bodyFrameReader = null;

        /// <summary>
        /// Array for the bodies
        /// </summary>
        private Body[] bodies = null;

        /// <summary>
        /// definition of bones
        /// </summary>
        private List<Tuple<JointType, JointType>> bones;

        /// <summary>
        /// Width of display (depth space)
        /// </summary>
        private int displayWidth;

        /// <summary>
        /// Height of display (depth space)
        /// </summary>
        private int displayHeight;

        /// <summary>
        /// List of colors for each body tracked
        /// </summary>
        private List<Pen> bodyColors;

        /// <summary>
        /// Current status text to display
        /// </summary>
        private string statusText = null;

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            //file2.WriteLine("-------------------hat Upore--------------------" + Environment.NewLine);

            // one sensor is currently supported
            this.kinectSensor = KinectSensor.GetDefault();
            featureVector = new List<List<double>>();

            // get the coordinate mapper
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;

            // get the depth (display) extents
            FrameDescription frameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;

            // get size of joint space
            this.displayWidth = frameDescription.Width;
            this.displayHeight = frameDescription.Height;

            // open the reader for the body frames
            this.bodyFrameReader = this.kinectSensor.BodyFrameSource.OpenReader();

            // a bone defined as a line between two joints
            this.bones = new List<Tuple<JointType, JointType>>();

            // Torso
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Head, JointType.Neck));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Neck, JointType.SpineShoulder));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.SpineMid));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineMid, JointType.SpineBase));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipLeft));

            // Right Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderRight, JointType.ElbowRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowRight, JointType.WristRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.HandRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandRight, JointType.HandTipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.ThumbRight));

            // Left Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderLeft, JointType.ElbowLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowLeft, JointType.WristLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.HandLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandLeft, JointType.HandTipLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.ThumbLeft));

            // Right Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipRight, JointType.KneeRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeRight, JointType.AnkleRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleRight, JointType.FootRight));

            // Left Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipLeft, JointType.KneeLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeLeft, JointType.AnkleLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleLeft, JointType.FootLeft));

            // populate body colors, one for each BodyIndex
            this.bodyColors = new List<Pen>();

            this.bodyColors.Add(new Pen(Brushes.Red, 6));
            this.bodyColors.Add(new Pen(Brushes.Orange, 6));
            this.bodyColors.Add(new Pen(Brushes.Green, 6));
            this.bodyColors.Add(new Pen(Brushes.Blue, 6));
            this.bodyColors.Add(new Pen(Brushes.Indigo, 6));
            this.bodyColors.Add(new Pen(Brushes.Violet, 6));

            // set IsAvailableChanged event notifier
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            // open the sensor
            this.kinectSensor.Open();

            // set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.NoSensorStatusText;

            // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();

            // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);

            // use the window object as the view model in this simple example
            this.DataContext = this;

            // initialize the components (controls) of the window
            this.InitializeComponent();
        }

        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;

        /// <summary>
        /// Gets the bitmap to display
        /// </summary>
        public ImageSource ImageSource
        {
            get
            {
                return this.imageSource;
            }
        }

        /// <summary>
        /// Gets or sets the current status text to display
        /// </summary>
        public string StatusText
        {
            get
            {
                return this.statusText;
            }

            set
            {
                if (this.statusText != value)
                {
                    this.statusText = value;

                    // notify any bound elements that the text has changed
                    if (this.PropertyChanged != null)
                    {
                        this.PropertyChanged(this, new PropertyChangedEventArgs("StatusText"));
                    }
                }
            }
        }

        /// <summary>
        /// Execute start up tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                this.bodyFrameReader.FrameArrived += this.Reader_FrameArrived;
            }
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                // BodyFrameReader is IDisposable
                this.bodyFrameReader.Dispose();
                this.bodyFrameReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        }

        ///*********************   CALCULATE ANGLE BETWEEN TWO VECTORS   *************************
        ////$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

        private double calcAngle(CameraSpacePoint position1, CameraSpacePoint position2, CameraSpacePoint position3, CameraSpacePoint position4)
        {
            float angle=0f, magnitude1, magnitude2, scalarProduct, magnitudeX_Y, temp;
            CameraSpacePoint vec1, vec2, normVec1, normVec2, vecX_Y;

            vec1.X = position1.X - position2.X;
            vec1.Y = position1.Y - position2.Y;
            vec1.Z = position1.Z - position2.Z;

            vec2.X = position3.X - position4.X;
            vec2.Y = position3.Y - position4.Y;
            vec2.Z = position3.Z - position4.Z;

            magnitude1 = (float) Math.Sqrt(vec1.X * vec1.X + vec1.Y * vec1.Y + vec1.Z * vec1.Z);
            magnitude2 = (float) Math.Sqrt(vec2.X * vec2.X + vec2.Y * vec2.Y + vec2.Z * vec2.Z);

            normVec1.X = vec1.X / magnitude1;
            normVec1.Y = vec1.Y / magnitude1;
            normVec1.Z = vec1.Z / magnitude1;

            normVec2.X = vec2.X / magnitude2;
            normVec2.Y = vec2.Y / magnitude2;
            normVec2.Z = vec2.Z / magnitude2;

            //scalarProduct = normVec1.X * normVec2.X + normVec1.Y * normVec2.Y + normVec1.Z * normVec2.Z;
            scalarProduct = vec1.X * vec2.X + vec1.Y * vec2.Y + vec1.Z * vec2.Z;

            temp = (float) scalarProduct / (magnitude1*magnitude2);
            angle = (float)Math.Acos(temp);

            //if (scalarProduct >= 0)
            //{
            //    vecX_Y.X = normVec1.X - normVec2.X;
            //    vecX_Y.Y = normVec1.Y - normVec2.Y;
            //    vecX_Y.Z = normVec1.Z - normVec2.Z;

            //    magnitudeX_Y = (float) Math.Sqrt(vecX_Y.X * vecX_Y.X + vecX_Y.Y * vecX_Y.Y + vecX_Y.Z * vecX_Y.Z);
            //    angle = (float) (2.0 * Math.Asin(magnitudeX_Y / 2.0));
            //}
            //else if (scalarProduct < 0)
            //{
            //    vecX_Y.X = -normVec1.X - normVec2.X;
            //    vecX_Y.Y = -normVec1.Y - normVec2.Y;
            //    vecX_Y.Z = -normVec1.Z - normVec2.Z;

            //    magnitudeX_Y = (float) Math.Sqrt(vecX_Y.X * vecX_Y.X + vecX_Y.Y * vecX_Y.Y + vecX_Y.Z * vecX_Y.Z);
            //    angle = (float) (180.0 - 2.0 * Math.Asin(magnitudeX_Y / 2.0));
            //}
            angle =(float) (180f / 3.1416f) * angle;

            return angle;
        }

        /// <summary>
        /// Handles the body frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            bool dataReceived = false;

            using (BodyFrame bodyFrame = e.FrameReference.AcquireFrame())
            {
                if (bodyFrame != null)
                {
                    if (this.bodies == null)
                    {
                        this.bodies = new Body[bodyFrame.BodyCount];
                    }

                    // The first time GetAndRefreshBodyData is called, Kinect will allocate each Body in the array.
                    // As long as those body objects are not disposed and not set to null in the array,
                    // those body objects will be re-used.
                    bodyFrame.GetAndRefreshBodyData(this.bodies);
                    dataReceived = true;
                }
            }
            
            //file2.WriteLine("----------------hat niche (হাত নিচে)--------------------" + Environment.NewLine);
            if (dataReceived)
            {
                using (DrawingContext dc = this.drawingGroup.Open())
                {
                    // Draw a transparent background to set the render size
                    dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                    //Debug.WriteLine("asdf");

                    int penIndex = 0;
                    foreach (Body body in this.bodies)
                    {
                        Pen drawPen = this.bodyColors[penIndex++];

                        if (body.IsTracked)
                        {
                            this.DrawClippedEdges(body, dc);

                            IReadOnlyDictionary<JointType, Joint> joints = body.Joints;

                            // convert the joint points to depth (display) space
                            Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();
                            CameraSpacePoint positionLeft = joints[JointType.HandLeft].Position;
                            CameraSpacePoint positionRight = joints[JointType.HandRight].Position;
                            CameraSpacePoint head = joints[JointType.Head].Position;
                            HandState handstateLeft = body.HandLeftState;
                            HandState handstateRight = body.HandRightState;
                            //string createText = "Hello and Welcome" + Environment.NewLine;
                            //cnt++;


                            //Console.WriteLine("position left: X-> " + positionLeft.X + "  Y-> " + positionLeft.Y + "   Z-> " + positionLeft.Z);
                            //Console.WriteLine("position right: X-> " + positionRight.X + "  Y-> " + positionRight.Y + "   Z-> " + positionRight.Z);
                            //Console.WriteLine("position head: X-> " + head.X + "  Y-> " + head.Y + "   Z-> " + head.Z);
                            if (gestureOn==false && handstateRight == HandState.Closed && (positionRight.Y > head.Y) && (positionRight.Y - head.Y) >= 0.10)
                            {
                                System.Threading.Thread.Sleep(500);
                                gestureOn = true;
                                cnt = 0;
                                //Debug.WriteLine("gesture started!");
                                Console.WriteLine("gesture started!");
                                featureVector.Clear();
                                continue;
                            }

                            else if (gestureOn == true && handstateLeft == HandState.Closed && (positionLeft.Y > head.Y) && (positionLeft.Y - head.Y) >= 0.10)
                            {
                                gestureOn = false;
                                Console.WriteLine("gesture ended!" + Environment.NewLine + "Feature Vector :" + Environment.NewLine);
                                foreach (var sublist in featureVector)
                                {
                                    foreach (var obj in sublist)
                                    {
                                        Console.WriteLine(obj + " ");
                                    }
                                    Console.WriteLine(Environment.NewLine);
                                }
                            }

                            //if (gestureOn == false && positionLeft.X >=(double) -0.50 && positionRight.X <= (double)0.60 && 
                            //    positionLeft.Y >= (double)0.15 && positionLeft.Y <= (double)0.50 &&
                            //    positionRight.Y >= (double)0.15 && positionRight.Y <= (double)0.50 )
                            //{
                            //    gestureOn = true;
                            //    cnt = 0;
                            //    //Debug.WriteLine("gesture started!");
                            //    Console.WriteLine("gesture started!");
                            //    featureVector.Clear();
                            //    continue;
                            //}

                            //else if (gestureOn == true && positionLeft.X >= (double)-0.50 && positionRight.X <= (double)0.60 &&
                            //    positionLeft.Y >= (double)0.15 && positionLeft.Y <= (double)0.50 &&
                            //    positionRight.Y >= (double)0.15 && positionRight.Y <= (double)0.50)
                            //{

                            //}

                            if (gestureOn)
                            {
                                List<double> list = new List<double>();
                                CameraSpacePoint shoulderLeft = joints[JointType.ShoulderLeft].Position;
                                CameraSpacePoint elbowLeft = joints[JointType.ElbowLeft].Position;
                                CameraSpacePoint wristLeft = joints[JointType.WristLeft].Position;
                                CameraSpacePoint shoulderRight = joints[JointType.ShoulderRight].Position;
                                CameraSpacePoint elbowRight = joints[JointType.ElbowRight].Position;
                                CameraSpacePoint wristRight = joints[JointType.WristRight].Position;
                                CameraSpacePoint spineShoulder = joints[JointType.SpineShoulder].Position;
                                CameraSpacePoint spineBase = joints[JointType.SpineBase].Position;
                                CameraSpacePoint kneeLeft = joints[JointType.KneeLeft].Position;
                                CameraSpacePoint kneeRight = joints[JointType.KneeRight].Position;
                                CameraSpacePoint ancleLeft = joints[JointType.AnkleLeft].Position;
                                CameraSpacePoint ancleRight = joints[JointType.AnkleRight].Position;
                                CameraSpacePoint hipRight = joints[JointType.HipRight].Position;
                                CameraSpacePoint hipLeft = joints[JointType.HipLeft].Position;
                                
                                list.Add(calcAngle(shoulderLeft, elbowLeft, wristLeft, elbowLeft));//1
                                list.Add(calcAngle(shoulderRight, elbowRight, wristRight, elbowRight));//2
                                list.Add(calcAngle(spineShoulder, shoulderLeft, elbowLeft, shoulderLeft));//3
                                list.Add(calcAngle(spineShoulder, shoulderRight, elbowRight, shoulderRight));//4
                                list.Add(calcAngle(elbowLeft, shoulderLeft, elbowRight, shoulderRight));//5
                                list.Add(calcAngle(wristLeft, elbowLeft, wristRight, elbowRight));//6
                                list.Add(calcAngle(elbowLeft, shoulderLeft, spineBase, spineShoulder));//7
                                list.Add(calcAngle(spineBase, spineShoulder, elbowRight, shoulderRight));//8
                                list.Add(calcAngle(spineBase, spineShoulder, wristLeft, elbowLeft));//9
                                list.Add(calcAngle(spineBase, spineShoulder, wristRight, elbowRight));//10
                                list.Add(calcAngle(spineShoulder, spineBase, ancleLeft, kneeLeft));//11
                                list.Add(calcAngle(spineShoulder, spineBase, ancleRight, kneeRight));//12
                                list.Add(calcAngle(hipLeft, kneeLeft, ancleLeft, kneeLeft));//13
                                list.Add(calcAngle(hipRight, kneeRight, ancleRight, kneeRight));//14
                                list.Add(calcAngle(kneeLeft, hipLeft, spineShoulder, spineBase));//15
                                list.Add(calcAngle(kneeRight, hipRight, spineShoulder, spineBase));//16
                                list.Add(calcAngle(kneeRight, hipRight, kneeLeft, hipLeft));//17
                                list.Add(calcAngle(ancleRight, kneeRight, ancleLeft, kneeLeft));//18

                                //featureVector.Add(list);
                                //Debug.Write(FeatureVector[i] + " ");
                                //System.Threading.Thread.Sleep(200);
                                
                                file2.WriteLine("new angles:" + Environment.NewLine);
                                //file2.WriteLine(list[0] + " " + list[1]);
                                featureVector.Add(list);
                                //file2.WriteLine(featureVector[i++]);
                            }

                            //file2.WriteLine("frame " + cnt + ":" + Environment.NewLine);
                            //file2.WriteLine("left hand position -> " + positionLeft.X + " " + positionLeft.Y + " " + positionLeft.Z + Environment.NewLine);
                            ///file2.WriteLine("right hand position -> " + positionRight.X + " " + positionRight.Y + " " + positionRight.Z + Environment.NewLine);

                            foreach (JointType jointType in joints.Keys)
                            {
                                // sometimes the depth(Z) of an inferred joint may show as negative
                                // clamp down to 0.1f to prevent coordinatemapper from returning (-Infinity, -Infinity)
                                CameraSpacePoint position = joints[jointType].Position;
                                //File.WriteAllText("D:\\kinect\\BodyBasics-WPF\\output.txt", createText);
                                
                                if (position.Z < 0)
                                {
                                    position.Z = InferredZPositionClamp;
                                }

                                DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position);
                                jointPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                            }

                            this.DrawBody(joints, jointPoints, dc, drawPen);

                            this.DrawHand(body.HandLeftState, jointPoints[JointType.HandLeft], dc);
                            this.DrawHand(body.HandRightState, jointPoints[JointType.HandRight], dc);
                        }
                    }

                    // prevent drawing outside of our render area
                    this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                    
                }
            }
            
        }

        /// <summary>
        /// Draws a body
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="drawingPen">specifies color to draw a specific body</param>
        private void DrawBody(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, DrawingContext drawingContext, Pen drawingPen)
        {
            // Draw the bones
            foreach (var bone in this.bones)
            {
                this.DrawBone(joints, jointPoints, bone.Item1, bone.Item2, drawingContext, drawingPen);
            }

            // Draw the joints
            foreach (JointType jointType in joints.Keys)
            {
                Brush drawBrush = null;

                TrackingState trackingState = joints[jointType].TrackingState;

                if (trackingState == TrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;
                }
                else if (trackingState == TrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;
                }

                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, jointPoints[jointType], JointThickness, JointThickness);
                }
            }
        }

        /// <summary>
        /// Draws one bone of a body (joint to joint)
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="jointType0">first joint of bone to draw</param>
        /// <param name="jointType1">second joint of bone to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// /// <param name="drawingPen">specifies color to draw a specific bone</param>
        private void DrawBone(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, JointType jointType0, JointType jointType1, DrawingContext drawingContext, Pen drawingPen)
        {
            Joint joint0 = joints[jointType0];
            Joint joint1 = joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == TrackingState.NotTracked ||
                joint1.TrackingState == TrackingState.NotTracked)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if ((joint0.TrackingState == TrackingState.Tracked) && (joint1.TrackingState == TrackingState.Tracked))
            {
                drawPen = drawingPen;
            }

            drawingContext.DrawLine(drawPen, jointPoints[jointType0], jointPoints[jointType1]);
        }

        /// <summary>
        /// Draws a hand symbol if the hand is tracked: red circle = closed, green circle = opened; blue circle = lasso
        /// </summary>
        /// <param name="handState">state of the hand</param>
        /// <param name="handPosition">position of the hand</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawHand(HandState handState, Point handPosition, DrawingContext drawingContext)
        {
            switch (handState)
            {
                case HandState.Closed:
                    drawingContext.DrawEllipse(this.handClosedBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Open:
                    drawingContext.DrawEllipse(this.handOpenBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Lasso:
                    drawingContext.DrawEllipse(this.handLassoBrush, null, handPosition, HandSize, HandSize);
                    break;
            }
        }

        /// <summary>
        /// Draws indicators to show which edges are clipping body data
        /// </summary>
        /// <param name="body">body to draw clipping information for</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawClippedEdges(Body body, DrawingContext drawingContext)
        {
            FrameEdges clippedEdges = body.ClippedEdges;

            if (clippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, this.displayHeight - ClipBoundsThickness, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, this.displayHeight));
            }

            if (clippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(this.displayWidth - ClipBoundsThickness, 0, ClipBoundsThickness, this.displayHeight));
            }
        }

        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            // on failure, set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.SensorNotAvailableStatusText;
        }
    }
    //file2.Close();
}
