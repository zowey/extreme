/*  Usage and description
 *  
 *  1. SKELETON JOINTS
 *     - apropriate model joints must be connected into these Transform properties
 * 
 *
 *  2. SKELETON TRANSLATION
 * 	   - 'mTranslationJoint' must be connected with root joint transform on model (root hips for example)
 * 	   - only one type of translation can be active 
 *       ( translationOfSkeleton 
 *         or translationAccordingToSpine 
 *         or translationAccordingToHips 
 *       )
 * 	   - 'scaleOrTranslateInZAxis' turns on or off translation in z - axis. 
 *        .. False turns on translation,
 * 	      .. True turns off translation in depth, and turns on scaling of model depending on user proximity
 * 	   - Translation in Z Axis can be estimated on different ways, depends on which option is turned on:
 *       ( scaleWristElbow 
 *         or scaleTorso 
 *         or scaleSpineShoulder 
 *         or scaleHeadSpine 
 *        )
 * 	   - 'translationScale' is vector which determines intensity of translation on each axis
 * 
 * 
 *  3. SKELETON SCALE (z translation)
 * 	   - This option is turned on with 'scaleOrTranslateInZAxis'
 *     - only one type of scale can be active 
 *       ( scaleWristElbow 
 *         or scaleTorso 
 *         or scaleSpineShoulder 
 *         or scaleHeadSpine 
 *        ) by 'relativeBoneForScaling'
 * 
 *
 *  4. FILTERING
 *     - Type of filter: 
 *     (a) Exponentially Weighted Moving Average - EWMA
 * 	   (b) Double Exponential Smoothing Filter - DESF
 * 	
 * 	   - 'alphaDE' - dampening factor
 *        A small α gives larger weight to older input samples, and hence, the effect of older inputs is larger
 * 	   - 'gamaDE' - controls how sensitive the trends is to recent changes in input
 *        A large gamaDE results in less latency in trend; that is, the trend element follows 
 *        the recent changes in input faster, while a small
 *        gamaDE gives larger weight to older input samples and, hence, results in longer delay in trend elements 
 *        catching up with changes in input. We can think gamaDE as the dampening factor used in exponential filtering of joint velocity
 *     - 'HConst' (Exponential averaging mean smoothing filter) should be epirically determined (0.1 - ? )
 *     - 'WindowSize' is size of buffered values used for smoothing/filtering ( 5 - 10 for EWMA, 3 is minimum for DESF ) 
 * 
 *
 *  5. JOINT ROTATION
 * 	   - 'transmissionOnHead' 
 *       if the head rotation needs to be emphasised this should be turned on. This is actually hack, 
 *       because in tracking skeleton there is no neck joint, so rotation should be passed
 *       on the neck as well as on the head
 *       Better to leave it turned on, otherwise head wont move
 */

using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using Xtr3D.Net.ExtremeMotion;
using Xtr3D.Net.ColorImage;
using Xtr3D.Net.ExtremeMotion.Data;
using Xtr3D.Net.ExtremeMotion.Interop.Types;

public class JointsUpdate : MonoBehaviour {
	
	
	// 1. SKELETON JOINTS
	
	public Transform mHead;
	public Transform mSpine;
	public Transform mHandL;
	public Transform mHandR;
	public Transform mElbowR;
	public Transform mElbowL;
	public Transform mShoulderL;
	public Transform mShoulderR;
	public Transform mHipCenter;
	public Transform mShoulderCenter;
	
	
	// 2. SKELETON TRANSLATION
	
	public bool translationOfSkeletonActive = true;
	public Transform mTranslationJoint; 
	
	public enum XYtranslationEstimationType
	{
		AccordingToSpine,
		AccordingToHips,
	}

	public XYtranslationEstimationType XYtranslationEstimationModelType;

	public Vector3 translationScale = new Vector3(1,1,1);
	
	private bool firstDataCapture;
	private bool firstTranslationData;	
	private Vector3 rootHipFirstPosition;
	private Vector3 rootHipDelta;
	private Vector3 pelvisFirstPosition;
	private float firstDistanceFromCamera;
	
	// 3. SKELETON SCALE
	
	public GameObject model;

	public enum scaleType
	{
		WristElbow,
		Torso,
		SpineShoulder,
		HeadSpine
	}
	
	public scaleType relativeBoneForScaling;
	public bool scaleOrTranslateInZAxis = false;   // If user proximity estimation will be used for scaling or translating

	private Vector3 firstCaptureSizeOfRelativeBone;
	private int DataCaptureIndex;


	
	// 4. FILTERING
	
	public enum filterType
	{
		ExponentiallyWeightedMovingAverage,
		DoubleExponentialSmoothingFilter
	}

	public filterType filtertype;

	public float HConst = 0.1f; // For smooting vector, it is empirically distinct
	public int windowSizeK = 5; // For smoothing vector

	private Vector3 [] vectorWindow;	// For smoothing vector

	// Double exponential
	public float alphaDE = 0.1f; // dampening factor, empirically distinct
	public float gamaDE = 0.1f;  // controls how sensitive the trends is to recent changes in input, , empirically distinct
	
	
	// 5. JOINT ROTATION
	
	public bool transmissionOnHead = true;
	private Quaternion[] baseRotation; // Starting orientation of the joints
	private Vector3[] boneDirection;   // In the bone's local space, the direction of the bones
	
	
	// Extreme M. data
	
	private const float HEIGHT_MULTIPLIER = 2; // Value should be 2 when using ortographic camera in unity
	private const int DEPTH_CONSTANT = 1; // Value should be 2 when using ortographic camera in unity
	// Value shoud be 0 when using perspective camera in unity
	
	private float textureXPos;
	private float textureYPos;
	private float textureDimensionX;
	private float textureDimensionY;
	
	public GameObject myCube;
	public GUIText guiTextComponent;
	public LineRenderer lineRenderer;
	
	private FrameRateCalc frameRateCalc;
	
	public GameObject joint;
	
	private Dictionary<JointType, GameObject> jointsGameObjects = new Dictionary<JointType, GameObject>();
	private Dictionary<JointType, Xtr3D.Net.ExtremeMotion.Data.Joint> typesToJoints = new Dictionary<JointType, Xtr3D.Net.ExtremeMotion.Data.Joint>();
	
	/*
	 *  null, Hip_Center, Spine, Shoulder_Center,
			Collar_Left, Shoulder_Left, Elbow_Left, Wrist_Left,
			Collar_Right, Shoulder_Right, Elbow_Right, Wrist_Right,
			Hip_Override, Hip_Left, Knee_Left, Ankle_Left,
			null, Hip_Right, Knee_Right, Ankle_Right,
			//extra joints to determine the direction of some bones
			Head, Hand_Left, Hand_Right, Foot_Left, Foot_Right};
	 */
	
	// Poredano koliko toliko po hijerarhiji da se lakse dohvaca prethodnik za izracun orjentacije zglobova
	private JointType[] jointTypesArray = new JointType[]
	{
		JointType.HipCenter,
		JointType.Spine,
		JointType.ShoulderCenter,
		JointType.Head,
		JointType.ShoulderLeft,
		JointType.ElbowLeft, 
		JointType.HandLeft,
		JointType.ShoulderRight, 
		JointType.ElbowRight,
		JointType.HandRight
	};
	
	private Dictionary<JointType, Transform> jointsModel = new Dictionary<JointType, Transform>();
	
	private Dictionary<JointType, Vector3[]> jointsPositionBuffer = new Dictionary<JointType, Vector3[]>();
	private Dictionary<JointType, Vector3[]> textureJointsPositionBufferS = new Dictionary<JointType, Vector3[]>();
	private Dictionary<JointType, Vector3[]> textureJointsPositionBufferT = new Dictionary<JointType, Vector3[]>();
	
	private Dictionary<JointType, short> jointsLastIndex = new Dictionary<JointType, short>();
	private Queue<Vector3> positionsBuffer = new Queue<Vector3>();
	
	
	private Dictionary<JointType, Vector3> jointsModelFiltered = new Dictionary<JointType, Vector3>();
	private Dictionary<JointType, Vector3> textureJointsFiltered = new Dictionary<JointType, Vector3>();

	// Double exponential filter helper 1D buffer
	private Dictionary<JointType, Vector3> trend = new Dictionary<JointType, Vector3>();
	private Dictionary<JointType, Vector3> prevSmoothedVector = new Dictionary<JointType, Vector3>();
	
	private Dictionary<JointType, int> indexOfJointType = new Dictionary<JointType, int>();
	
	
	long lastFrameID = -1;
	long currFrameID = -1;
	
	
	void Start () 
	{
		// Extreme m. 
		CalculateTextureParams();
		CreateJoints();
		Xtr3dGeneratorStateManager.RegisterDataCallback(MyDataFrameReady);
		// init frameRateCalc for calculating avarage running fps in the last given frames
		frameRateCalc = new FrameRateCalc(50);
		
		// For scaling and translating
		firstDataCapture = true;
		DataCaptureIndex = 0;
		firstTranslationData = true;
		// Debug.Log ("Iznos HConst = "+HConst);
		
	}
	
	
	private void CalculateTextureParams()
	{
		float heightMeasure = Camera.main.orthographicSize * HEIGHT_MULTIPLIER; // Calculating the current world view height measure
		textureDimensionX = Math.Abs(myCube.transform.localScale.x * (Screen.height/heightMeasure)); // calculating current cube size accroding to current screen resolution
		textureDimensionY = Math.Abs(myCube.transform.localScale.y * (Screen.height/heightMeasure)); // calculating current cube size accroding to current screen resolution
		Vector3 cubePos = Camera.main.WorldToScreenPoint(myCube.transform.position);
		textureXPos = cubePos.x - textureDimensionX/2;
		textureYPos = cubePos.y + textureDimensionY/2;
	}
	

	private void CreateJoints()
	{

		// COLOR MARKED SPHERES 

		foreach (JointType type in jointTypesArray) 
		{
			jointsGameObjects[type] = (GameObject) Instantiate(joint,new Vector3(0f,0f,-5),Quaternion.identity);
			if (type == JointType.Spine)
				jointsGameObjects[type].renderer.material.color = Color.red;
			if (type == JointType.HipCenter)
				jointsGameObjects[type].renderer.material.color = Color.green;
			if (type == JointType.ShoulderCenter)
				jointsGameObjects[type].renderer.material.color = Color.yellow;
			
		}
		
		
		// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		// DATA INITIALIZATION

		jointsModel[JointType.ShoulderCenter] = mShoulderCenter;
		jointsModel[JointType.Spine] 		  = mSpine;		
		jointsModel[JointType.HipCenter] 	  =	mHipCenter; 
		jointsModel[JointType.ShoulderLeft]   =	mShoulderL;
		jointsModel[JointType.ShoulderRight]  =	mShoulderR;
		jointsModel[JointType.ElbowLeft] 	  =	mElbowL;
		jointsModel[JointType.ElbowRight] 	  = mElbowR;		
		jointsModel[JointType.HandRight]	  =	mHandR;
		jointsModel[JointType.HandLeft] 	  =	mHandL;
		jointsModel[JointType.Head]           = mHead; 	
		
		
		jointsLastIndex[JointType.ShoulderCenter] 	= 0;
		jointsLastIndex[JointType.Spine] 		  	= 0;
		jointsLastIndex[JointType.HipCenter] 	  	= 0;
		jointsLastIndex[JointType.ShoulderLeft]   	= 0;
		jointsLastIndex[JointType.ShoulderRight]  	= 0;
		jointsLastIndex[JointType.ElbowLeft] 	  	= 0;
		jointsLastIndex[JointType.ElbowRight] 		= 0;
		jointsLastIndex[JointType.HandRight]	  	= 0;
		jointsLastIndex[JointType.HandLeft] 	  	= 0;
		jointsLastIndex[JointType.Head]           	= 0;
		
		
		// Window of positions
		
		jointsPositionBuffer.Add(JointType.ShoulderCenter, new Vector3[windowSizeK+1]);
		jointsPositionBuffer.Add(JointType.Spine, new Vector3[windowSizeK+1]);
		jointsPositionBuffer.Add(JointType.HipCenter, new Vector3[windowSizeK+1]);
		jointsPositionBuffer.Add(JointType.ShoulderLeft, new Vector3[windowSizeK+1]);
		jointsPositionBuffer.Add(JointType.ShoulderRight, new Vector3[windowSizeK+1]);
		jointsPositionBuffer.Add(JointType.ElbowLeft, new Vector3[windowSizeK+1]);
		jointsPositionBuffer.Add(JointType.ElbowRight, new Vector3[windowSizeK+1]);
		jointsPositionBuffer.Add(JointType.HandRight, new Vector3[windowSizeK+1]);
		jointsPositionBuffer.Add(JointType.HandLeft, new Vector3[windowSizeK+1]);
		jointsPositionBuffer.Add(JointType.Head, new Vector3[windowSizeK+1]);
		
		textureJointsPositionBufferS.Add(JointType.Spine, new Vector3[windowSizeK+1]);
		textureJointsPositionBufferS.Add(JointType.HipCenter, new Vector3[windowSizeK+1]);
		textureJointsPositionBufferS.Add(JointType.HandLeft, new Vector3[windowSizeK+1]);
		textureJointsPositionBufferS.Add(JointType.ElbowLeft, new Vector3[windowSizeK+1]);
		textureJointsPositionBufferS.Add(JointType.ShoulderLeft, new Vector3[windowSizeK+1]);
		textureJointsPositionBufferS.Add(JointType.Head, new Vector3[windowSizeK+1]);
		
		textureJointsPositionBufferT.Add(JointType.Spine, new Vector3[windowSizeK+1]);
		textureJointsPositionBufferT.Add(JointType.HipCenter, new Vector3[windowSizeK+1]);
		
		textureJointsFiltered.Add(JointType.HandRight, new Vector3 (0,0,0));
		textureJointsFiltered.Add(JointType.ElbowRight, new Vector3 (0,0,0));
		textureJointsFiltered.Add(JointType.Spine, new Vector3 (0,0,0));
		textureJointsFiltered.Add(JointType.HipCenter, new Vector3 (0,0,0));
		textureJointsFiltered.Add (JointType.ShoulderRight, new Vector3(0,0,0));
		textureJointsFiltered.Add (JointType.Head, new Vector3(0,0,0));
		
		jointsModelFiltered.Add(JointType.ShoulderCenter, new Vector3 (0,0,0));
		jointsModelFiltered.Add(JointType.Spine, new Vector3 (0,0,0));
		jointsModelFiltered.Add(JointType.HipCenter, new Vector3 (0,0,0));
		jointsModelFiltered.Add(JointType.ShoulderLeft, new Vector3 (0,0,0));
		jointsModelFiltered.Add(JointType.ShoulderRight, new Vector3 (0,0,0));
		jointsModelFiltered.Add(JointType.ElbowLeft, new Vector3 (0,0,0));
		jointsModelFiltered.Add(JointType.ElbowRight, new Vector3 (0,0,0));
		jointsModelFiltered.Add(JointType.HandRight, new Vector3 (0,0,0));
		jointsModelFiltered.Add(JointType.HandLeft, new Vector3 (0,0,0));
		jointsModelFiltered.Add(JointType.Head, new Vector3 (0,0,0));

		trend.Add(JointType.ShoulderCenter, new Vector3 (0,0,0));
		trend.Add(JointType.Spine, new Vector3 (0,0,0));
		trend.Add(JointType.HipCenter, new Vector3 (0,0,0));
		trend.Add(JointType.ShoulderLeft, new Vector3 (0,0,0));
		trend.Add(JointType.ShoulderRight, new Vector3 (0,0,0));
		trend.Add(JointType.ElbowLeft, new Vector3 (0,0,0));
		trend.Add(JointType.ElbowRight, new Vector3 (0,0,0));
		trend.Add(JointType.HandRight, new Vector3 (0,0,0));
		trend.Add(JointType.HandLeft, new Vector3 (0,0,0));
		trend.Add(JointType.Head, new Vector3 (0,0,0));

		prevSmoothedVector.Add(JointType.ShoulderCenter, new Vector3 (0,0,0));
		prevSmoothedVector.Add(JointType.Spine, new Vector3 (0,0,0));
		prevSmoothedVector.Add(JointType.HipCenter, new Vector3 (0,0,0));
		prevSmoothedVector.Add(JointType.ShoulderLeft, new Vector3 (0,0,0));
		prevSmoothedVector.Add(JointType.ShoulderRight, new Vector3 (0,0,0));
		prevSmoothedVector.Add(JointType.ElbowLeft, new Vector3 (0,0,0));
		prevSmoothedVector.Add(JointType.ElbowRight, new Vector3 (0,0,0));
		prevSmoothedVector.Add(JointType.HandRight, new Vector3 (0,0,0));
		prevSmoothedVector.Add(JointType.HandLeft, new Vector3 (0,0,0));
		prevSmoothedVector.Add(JointType.Head, new Vector3 (0,0,0));
		
		// Promatrani zglobovi:
		// ----------------------------------------------------------------------------------------------
		// 0 JointType.HipCenter, 		-- ne
		// 1 JointType.Spine, 			-- da -- kriticno
		// 2 JointType.ShoulderCenter,	-- da -- kriticno
		// 3 JointType.Head, 			-- da -- kriticno
		// 4 JointType.ShoulderLeft, 	-- ne
		// 5 JointType.ElbowLeft, 		-- da
		// 6 JointType.HandLeft, 		-- da
		// 7 JointType.ShoulderRight, 	-- ne
		// 8 JointType.ElbowRight, 		-- da
		// 9 JointType.HandRight 		-- da
		
		indexOfJointType.Add(JointType.HipCenter, 0);
		indexOfJointType.Add(JointType.Spine, 1);
		indexOfJointType.Add(JointType.ShoulderCenter, 2);		
		indexOfJointType.Add(JointType.Head, 3);
		indexOfJointType.Add(JointType.ShoulderLeft, 4);
		indexOfJointType.Add(JointType.ElbowLeft, 5);
		indexOfJointType.Add(JointType.HandLeft, 6);
		indexOfJointType.Add(JointType.ShoulderRight, 7);
		indexOfJointType.Add(JointType.ElbowRight, 8);
		indexOfJointType.Add(JointType.HandRight, 9);
		
		
		skeletonInit();
		
		// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	}
	
	
	private void MyDataFrameReady(object sender, Xtr3D.Net.ExtremeMotion.Data.DataFrameReadyEventArgs e)
	{
		using (var dataFrame = e.OpenFrame() as DataFrame)
		{
			if (dataFrame!=null)
			{
				currFrameID = dataFrame.FrameKey.FrameNumberKey;
				if (currFrameID <= lastFrameID  && currFrameID!=1) //currFrameId=1 on reset/resume!!!
					return;
				
				lastFrameID = currFrameID;
				
				//update frameRateCalc, we need to call this every frame as we are calculating avarage fps in the last x frames.
				frameRateCalc.UpdateAvgFps();
				JointCollection skl = dataFrame.Skeletons[0].Joints;
				
				//use a copy of the Joints data structure as the dataFrame values can change.
				typesToJoints[JointType.ShoulderCenter] = skl.ShoulderCenter;
				typesToJoints[JointType.Spine] 			= skl.Spine;
				typesToJoints[JointType.HipCenter] 		= skl.HipCenter;
				typesToJoints[JointType.ShoulderLeft] 	= skl.ShoulderLeft;
				typesToJoints[JointType.ShoulderRight]	= skl.ShoulderRight;
				typesToJoints[JointType.ElbowLeft] 		= skl.ElbowLeft;
				typesToJoints[JointType.ElbowRight] 	= skl.ElbowRight;
				typesToJoints[JointType.HandRight]		= skl.HandRight;
				typesToJoints[JointType.HandLeft] 		= skl.HandLeft;
				typesToJoints[JointType.Head] 			= skl.Head;
				
				
			}
		}
		
	}
	

	void OnGUI () 
	{
		//display in our text component the avg fps
		guiTextComponent.text = System.String.Format("{0:F2} Skeleton FPS",frameRateCalc.GetAvgFps());
	}
	

	void Update()
	{
		// don`t do anything if no info was passed yet
		if (!typesToJoints.ContainsKey(JointType.Head))
			return;
		
		foreach (JointType type in jointTypesArray)
		{
			
			// DEBUG
			// To find out order of traversing joints
			// Debug.Log ("Current joint = "+type);
			
			if (typesToJoints [type].jointTrackingState == JointTrackingState.Tracked)
			{
				
				float x = typesToJoints[type].skeletonPoint.X;
				float y = typesToJoints[type].skeletonPoint.Y;
				float z = -Mathf.Abs(typesToJoints[type].skeletonPoint.Z);
				
				
				jointsGameObjects[type].SetActive(true);
				jointsGameObjects[type].transform.position = new Vector3(x, y, z);
				
				// Debug.Log ("Index = "+jointsLastIndex[type]);
				
				
				
				if (jointsLastIndex[type] == windowSizeK) 
				{
					// 1. FILTERING FOR JOINT ROTATION ----------------------------------------------------------
					
					jointsPositionBuffer[type][jointsLastIndex[type]] = new Vector3(x,y,z);
					var tmpJointsPosBuffer = jointsPositionBuffer[type];

					if (filtertype == filterType.ExponentiallyWeightedMovingAverage)
						jointsModelFiltered[type] = smoothVector(ref tmpJointsPosBuffer, windowSizeK); 
					if (filtertype == filterType.DoubleExponentialSmoothingFilter) 
						jointsModelFiltered[type] = smoothVectorDE(jointsPositionBuffer[type][jointsLastIndex[type]], type);
					


					
					// 2. FILTERING FOR SCALING -----------------------------------------------------------------

					bool bufferFilled = true;

					switch(relativeBoneForScaling) 
					{
					case scaleType.WristElbow : 
						if (type == JointType.HandRight || type == JointType.ElbowRight) 
							scaleFiltering(type, bufferFilled);
						break;
							
					case scaleType.Torso : 
						if (type == JointType.Spine || type == JointType.HipCenter) 
							scaleFiltering(type, bufferFilled);
						break;
							
					case scaleType.SpineShoulder : 
						if (type == JointType.Spine || type == JointType.ShoulderRight) 
							scaleFiltering(type, bufferFilled);		
						break;

					case scaleType.HeadSpine :
						if (type == JointType.Spine || type == JointType.Head) 
							scaleFiltering(type, bufferFilled);
						break;

					default : 
						if (type == JointType.Spine || type == JointType.ShoulderRight) 
							scaleFiltering(type, bufferFilled);	
						break;
					}

					
					// 3. FILTERING FOR TRANSLATION --------------------------------------------------------------
					
					if (translationOfSkeletonActive) {
						
						JointType typeT = JointType.Spine;
						
						if ( XYtranslationEstimationModelType == XYtranslationEstimationType.AccordingToSpine ) 
							typeT = JointType.Spine;

						if ( XYtranslationEstimationModelType == XYtranslationEstimationType.AccordingToHips )
							typeT = JointType.HipCenter;
						
						if ( type == typeT ) 
						{
							// Translation is estimated from texture coordinates because they are absolute
							// and raw data are relative to some unknown point on skeleton
							
							// Scaling and translating texture coordinates
							// to match world coordinates approximately, offset between -1 and 1 meters
							// + Buffering, smoothing
							float xT = (typesToJoints[type].skeletonPoint.ImgCoordNormHorizontal)*2 - 1;
							float yT = (-1* typesToJoints[type].skeletonPoint.ImgCoordNormVertical)*2 + 1;
							
							// TRANSLATION IN Z AXIS ACCORDING TO SIZE OF THE RELATIVE BONE
							// New Distance From Camera = starting Distance From Camera * scale
							// It is neccessary to meassure starting Distance From Camera
							float zT = DEPTH_CONSTANT;

							if (scaleOrTranslateInZAxis == false) // TRANSLATE IN Z AXIS to demonstrate user proximity
							{
								if (firstTranslationData) 
									firstDistanceFromCamera = Camera.main.transform.position.z - pelvisFirstPosition.z;


								zT = calculateTranslationInZ (relativeBoneForScaling); 

							}
							
							
							Debug.Log (">>>>>>>>>>>>>>>> deltaDistance ="+zT);
							
							textureJointsPositionBufferT[type][jointsLastIndex[type]] = new Vector3 (xT, yT, zT);
							
							var tmpTBuffer = textureJointsPositionBufferT[type];
							
							Vector3 tt = smoothVector(ref tmpTBuffer, windowSizeK);
							
							
							// Take pivot, for relative estimation of offset
							if (firstTranslationData) 
							{
								firstTranslationData = false;
								rootHipFirstPosition = new Vector3(tt.x, tt.y, tt.z);
								pelvisFirstPosition = mTranslationJoint.position;
							}
							
							// Offset, delta
							rootHipDelta = new Vector3(tt.x, tt.y, tt.z) - rootHipFirstPosition;
							
							Debug.Log ("===============> tt.z ="+tt.z);
							
							// Apply offset on model root joint
							mTranslationJoint.position = new Vector3(
								pelvisFirstPosition.x+rootHipDelta.x*translationScale.x, 
								mTranslationJoint.position.y,
								pelvisFirstPosition.z+rootHipDelta.z*translationScale.z);
							
							/*

							Debug.Log (">>>>>>>>>>>>>>>>Root hips position = ("+tt.x+", "+tt.y+", "+tt.z+")");
							Debug.Log (">>>>>>>>>>>>>>>>rootHipDelta = ("+rootHipDelta.x+", "+rootHipDelta.y+", "+rootHipDelta.z+")");
							Debug.Log (">>>>>>>>>>>>>>>>mTranslationJoint= ("+mTranslationJoint.position.x+", "+mTranslationJoint.position.y+", "
							           +mTranslationJoint.position.z+")");
							*/
						}
						
					}

					

					RotateJoint (type);
					
					// Buffer shifting for estimating smoothing vector

					for (int i=0; i<windowSizeK; ++i) 
						jointsPositionBuffer[type][i] = jointsPositionBuffer[type][i+1];
					
					
					// MODEL SCALING for depth translation when ortographic camera
					// is in use to demonstrate user proximity

					if (scaleOrTranslateInZAxis) 
					{	
						float ratio = calculateTrackedSkltModelSkltRatio (relativeBoneForScaling);
						model.transform.localScale = new Vector3(ratio, ratio, ratio);
					}
					
					
				} else { // Buffer Window not filled yet, buffer < windowSizeK
				
					
					// 1. FILTERING FOR ROTATIONS ---------------------------------------------------------------

					jointsPositionBuffer[type][jointsLastIndex[type]] = new Vector3(x,y,z);
				

					if (filtertype == filterType.DoubleExponentialSmoothingFilter) 
					{ // Double exponential filter
						if (jointsLastIndex[type] == 1) 
							smoothVectorDEInit(jointsPositionBuffer[type][1], jointsPositionBuffer[type][0], type);


						if (jointsLastIndex[type] > 1) 
							jointsModelFiltered[type] = smoothVectorDE(jointsPositionBuffer[type][jointsLastIndex[type]], type);

					}

					++jointsLastIndex[type];

					

					// 2. FILTERING FOR SCALING -----------------------------------------------------------------

					bool bufferFilled = false;

					switch(relativeBoneForScaling) 
					{
					case scaleType.WristElbow : 
						if (type == JointType.HandRight || type == JointType.ElbowRight) 
							scaleFiltering(type, bufferFilled);
						break;
						
					case scaleType.Torso : 
						if (type == JointType.Spine || type == JointType.HipCenter) 
							scaleFiltering(type, bufferFilled);
						break;
						
					case scaleType.SpineShoulder : 
						if (type == JointType.Spine || type == JointType.ShoulderRight) 
							scaleFiltering(type, bufferFilled);		
						break;
						
					case scaleType.HeadSpine :
						if (type == JointType.Spine || type == JointType.Head) 
							scaleFiltering(type, bufferFilled);
						break;
						
					default : 
						if (type == JointType.Spine || type == JointType.ShoulderRight) 
							scaleFiltering(type, bufferFilled);	
						break;
					}
					
					
					// 3. FILTERING FOR TRANSLATION -------------------------------------------------------------
					
					if (translationOfSkeletonActive) 
					{
						
						JointType typeT = JointType.Spine;
						
						if ( XYtranslationEstimationModelType == XYtranslationEstimationType.AccordingToSpine ) 
							typeT = JointType.Spine;

						if ( XYtranslationEstimationModelType == XYtranslationEstimationType.AccordingToHips )
							typeT = JointType.HipCenter;
						
						if (type == typeT) 
						{
							// Translaciju gledamo iz teksturnih koordinata jer su one apsolutne
							// a sirovi podaci su relativni 
							
							// Treba sad skalirati i translatirati teksturne koordinate 
							// Da otprilike odgovaraju svjetovnim koordinatama, neki pomak unutar plus minus metra
							// Buffering, smoothing
							float xT = (typesToJoints[type].skeletonPoint.ImgCoordNormHorizontal)*2 - 1;
							float yT = (-1* typesToJoints[type].skeletonPoint.ImgCoordNormVertical)*2 + 1;
							float zT = DEPTH_CONSTANT;
							
							
							textureJointsPositionBufferT[type][jointsLastIndex[type]] = new Vector3 (xT, yT, zT);
						}
						
					}
					
					
				} 
				
				
			}
			else // joint is not tracked, Extreme Motion spheres are disabled then 
			{
				jointsGameObjects[type].SetActive(false);
			}
		}
	}
	

	/**
	 * smoothVector
	 *
	 * (Exponentially Weighted Moving Average - EWMA filter)
	 *
	 * Vector t is size of k+1 , t[k] is current observed vector for smoothing
	 */
	Vector3 smoothVector(ref Vector3[] t, int k) 
	{
		float wj;
		Vector3 sum1 = new Vector3(0,0,0);
		float sum2 = 0;
		float maxT = getMaxT(ref t, k);
		
		
		for (int j=0; j<=k; ++j) 
		{
			wj = vectorWeight(ref t, j, k, maxT);
			sum1 += wj * t[k-j];
			sum2 += wj; 
		}
		
		return (sum1 / sum2);
	}
	

	/**
	 * vectorWeight
	 *
	 * (Exponentially Weighted Moving Average - EWMA filter)
	 *
	 * Calculation of weight for every sample inside window
	 */
	float vectorWeight(ref Vector3[] t, int j, int k, float maxT) 
	{
		return (float)Math.Pow(Math.E, -j*HConst*maxT);
	}
	

	/**
	 * getMaxT
	 *
	 * (Exponentially Weighted Moving Average - EWMA filter)
	 *
	 * Calculate maximum difference between neighbour samples inside window
	 */
	float getMaxT(ref Vector3[] t, int k) 
	{
		float magnit;
		float max;
		Vector3 subtr;
		
		subtr = t[k] - t[k-1];
		max = subtr.magnitude;
		
		for (int l=2; l<=k; ++l) 
		{
			subtr = t[k] - t[k-l];
			
			if (subtr.magnitude > max) 
			{
				max = subtr.magnitude;
			}
		}
		
		return max;
	}


	/**
	 * smoothVectorDEInit
	 *
	 * (Double Exponential Smoothing Filter - DESF filter)
	 *
	 * Calculating smoothed vector initial value
	 * Used when second measurement is taken because ( X* = X1 - X0 )
	 *
	 */
	void smoothVectorDEInit (Vector3 X1, Vector3 X0, JointType type) {

		prevSmoothedVector[type] = X0;
		trend[type] = X1 - X0;

	}


	/**
	 * smoothVectorDE
	 *
	 * (Double Exponential Smoothing Filter - DESF filter)
	 *
	 * Calculating smoothed vector 
	 * Used when third and higher samples is taken 

	 * Including the Trend helps to reduce the delay as the filter fits a line to local input data,
	 * where the Trend is the slope of this fitted line
	 *
	 */
	Vector3 smoothVectorDE ( Vector3 Xn, JointType type) {

		Vector3 smoothedVector;
		smoothedVector = alphaDE * Xn + ( 1-alphaDE )*( prevSmoothedVector[type] + trend[type] );	
					
		// trend for next iteration		
		trend[type] = gamaDE*(smoothedVector - prevSmoothedVector[type]) + (1-gamaDE) * trend[type];
		prevSmoothedVector[type] = smoothedVector;

		return smoothedVector;

	}

	
	// ROTATION SECTION

	/**
	 *
	 * skeletionInit
	 *
	 * Storing the base rotations and bone directions (in bone-local space)
	 */
	void skeletonInit () 
	{

		/*

			JOINTS Hierarchy

			0 JointType.HipCenter, 		
			1 JointType.Spine, 			
			2 JointType.ShoulderCenter,	
			3 JointType.Head, 			
			4 JointType.ShoulderLeft, 	
			5 JointType.ElbowLeft, 		
			6 JointType.HandLeft, 		
			7 JointType.ShoulderRight, 	
			8 JointType.ElbowRight, 	
			9 JointType.HandRight 	

		*/

		baseRotation = new Quaternion[10];  // Ten joints
		boneDirection = new Vector3[9];     // Nine bones
		
		for (int i=0; i<10; ++i)
			baseRotation[i] = jointsModel[jointTypesArray[i]].localRotation;
		

		/* 

			BONES Hierarchy

			0 HipCenter -> Spine
			1 Spine -> ShoulderCenter
			2 ShoulderCenter -> Left Shoulder
			3 ShoulderCenter -> Right Shoulder
			4 ShoulderCenter -> Head
			5 Left Shoulder -> Left Elbow
			6 Right Shoulder -> Right Elbow
			7 Left Elbow -> Left Hand
			8 Right Elbow -> Right Hand

		*/
		boneDirection[0] = jointsModel[JointType.Spine].position - jointsModel[JointType.HipCenter].position; 
		boneDirection[1] = jointsModel[JointType.ShoulderCenter].position - jointsModel[JointType.Spine].position; 
		boneDirection[2] = jointsModel[JointType.Head].position - jointsModel[JointType.ShoulderCenter].position; 
		boneDirection[3] = jointsModel[JointType.ShoulderLeft].position - jointsModel[JointType.ShoulderCenter].position; 
		boneDirection[4] = jointsModel[JointType.ShoulderRight].position - jointsModel[JointType.ShoulderCenter].position; 
		boneDirection[5] = jointsModel[JointType.ElbowLeft].position - jointsModel[JointType.ShoulderLeft].position; 
		boneDirection[6] = jointsModel[JointType.ElbowRight].position - jointsModel[JointType.ShoulderRight].position; 
		boneDirection[7] = jointsModel[JointType.HandLeft].position - jointsModel[JointType.ElbowLeft].position; 
		boneDirection[8] = jointsModel[JointType.HandRight].position - jointsModel[JointType.ElbowRight].position; 

		/*
		for (int i=0; i<10; ++i)
		{
			
			baseRotation[i] = jointsModel[jointTypesArray[i]].localRotation;

			// Ako se ne radi o  zglobu, 
			// Kost postoji i ima ishodiste u prethodnom zglobu
			// A ponoriste se nalazi u trenutnom zglobu O---->O
			

			if (  jointTypesArray[i] != JointType.HandLeft  // Usmjerenje ovih kostiju koje ne postoje
			    && jointTypesArray[i] != JointType.HandRight) // ostati ce na (0,0,0)
				// && jointTypesArray[i] != JointType.Head)
			{
				
				if (jointTypesArray[i] == JointType.Head) 
				{ 
					if (transmissionOnHead) 
					{
						// ponavljamo usmjerenje za glavu isto koje je imalo s obzirom na kaljeznicu
						boneDirection[i] = jointsModel[jointTypesArray[i]].position 
							- jointsModel[jointTypesArray[i-1]].position;
						
						// Transformacija smjera kosti u lokalni koordinatni sustav
						boneDirection[i] = jointsModel[jointTypesArray[i]].InverseTransformDirection(boneDirection[i]);
					}
				} 
				else 
				{
					boneDirection[i] = jointsModel[jointTypesArray[i+1]].position
						- jointsModel[jointTypesArray[i]].position;
					
					// Transformacija smjera kosti u lokalni koordinatni sustav
					boneDirection[i] = jointsModel[jointTypesArray[i]].InverseTransformDirection(boneDirection[i]);
				}
			}
			
			
		} 
		*/
		
		
	}
	

	/**
	 *
	 * RotateJoint
	 *
	 * Joint rotation estimation 
	 * except for End effectors (right/left hand)
	 * Head is rotated according to previous bone (shoulderCenter -> Head)
	 *
	 */
	void RotateJoint (JointType type) 
	{

		/*

			JOINTS Hierarchy

			0 JointType.HipCenter, 		
			1 JointType.Spine, 			
			2 JointType.ShoulderCenter,	
			3 JointType.Head, 			
			4 JointType.ShoulderLeft, 	
			5 JointType.ElbowLeft, 		
			6 JointType.HandLeft, 		
			7 JointType.ShoulderRight, 	
			8 JointType.ElbowRight, 	
			9 JointType.HandRight 

		*/
		/* 

			BONES Hierarchy

			0 HipCenter -> Spine
			1 Spine -> ShoulderCenter
			2 ShoulderCenter -> Left Shoulder
			3 ShoulderCenter -> Right Shoulder
			4 ShoulderCenter -> Head
			5 Left Shoulder -> Left Elbow
			6 Right Shoulder -> Right Elbow
			7 Left Elbow -> Left Hand
			8 Right Elbow -> Right Hand

		*/		

		Vector3 target = new Vector3(0,0,0);;
		Vector3 direction = new Vector3(0,0,0);;
		int jointIndex = indexOfJointType[type];


		switch(type) {

			case JointType.HipCenter: 

				// target = new direction (towards new location), direction = first vector orientation
				target = jointPosition(JointType.Spine) - jointPosition(JointType.HipCenter);
				direction = boneDirection[0];

				RotateJointWithAngle (target, direction, type);

			break;

			case JointType.Spine: 

				// target = new direction (towards new location), direction = first vector orientation
				target = jointPosition(JointType.ShoulderCenter) - jointPosition(JointType.Spine);
				direction = boneDirection[1];

				RotateJointWithAngle (target, direction, type);

			break;

			case JointType.ShoulderCenter: 

				// Three cases, this bone defines rotation of three bones
				// target = new direction (towards new location), direction = first vector orientation

				target = jointPosition(JointType.ShoulderLeft) - jointPosition(JointType.ShoulderCenter);
				direction = boneDirection[2];
				RotateJointWithAngle (target, direction, type);

				target = jointPosition(JointType.ShoulderRight) - jointPosition(JointType.ShoulderCenter);
				direction = boneDirection[3];
				RotateJointWithAngle (target, direction, type);

				target = jointPosition(JointType.Head) - jointPosition(JointType.ShoulderCenter);
				direction = boneDirection[4];
				RotateJointWithAngle (target, direction, type);

			break;

			case JointType.Head: 

				if (transmissionOnHead) 
				{
					target = jointPosition(JointType.ShoulderCenter) - jointPosition(JointType.Head);
					target.x = target.x*3; // boost rotation a bit
					direction = boneDirection[jointIndex-1];

					RotateJointWithAngle (target, direction, type);
				}

			break;

			case JointType.ShoulderLeft: 

				target = jointPosition(JointType.ElbowLeft) - jointPosition(JointType.ShoulderLeft);
				direction = boneDirection[5];

				RotateJointWithAngle (target, direction, type);

			break;

			case JointType.ShoulderRight: 

				target = jointPosition(JointType.ElbowRight) - jointPosition(JointType.ShoulderRight);
				direction = boneDirection[6];

				RotateJointWithAngle (target, direction, type);

			break;

			case JointType.ElbowLeft: 

				target = jointPosition(JointType.HandLeft) - jointPosition(JointType.ElbowLeft);
				direction = boneDirection[7];

				RotateJointWithAngle (target, direction, type);

			break;

			case JointType.ElbowRight: 

				target = jointPosition(JointType.HandRight) - jointPosition(JointType.ElbowRight);
				direction = boneDirection[8];

				RotateJointWithAngle (target, direction, type);

			break;

			//case JointType.HandRight: target = jointPosition(JointType.Spine) - jointPosition(JointType.HandRight);
			//break;

			//case JointType.HandLeft: target = jointPosition(JointType.Spine) - jointPosition(JointType.HandLeft); 
			//break;
			
			default:
			break;

		}






		
		/*
		if (jointIndex != 9 && jointIndex != 6) // && jointIndex != 3){
		{
			if (jointIndex == 3) 
			{ // Za glavu ponavljamo rotaciju od shoulder_center do glave
				if (transmissionOnHead) 
				{
					target = jointPosition(jointIndex) - jointPosition(jointIndex-1);
					target.x = target.x*3;
					direction = boneDirection[jointIndex-1];
				}
			} 
			
			//else if (jointIndex == 2) // ++++++++++++++++++++++++++++++++++++++++++ Dodano 16.7.2014. +++++++
			//{ // Za shoulder center, njega rotiramo kako bi dobili dubinsko poziconiranje ramena
				// Kako ce rotacija shoulder centra premjestiti i lijevo rame i desno rame za isti iznos 
				// treba odrediti koje rame ce i zbog cega odredjivati taj iznos

			//	if (jointPosition(4).z > jointPosition(7).z) 
		//			target = jointPosition(4)-jointPosition(jointIndex);
			//	else
			//		target = jointPosition(7)-jointPosition(jointIndex);
				

			//} // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
			
			else 
			{
				target = jointPosition(jointIndex+1) - jointPosition(jointIndex);
			}
			
			// Debug.Log("Trenutno promatrani zglob = ("+jointIndex+")"+jointTypesArray[jointIndex]);
			
			
			// Rotation estimation
			target = transform.TransformDirection(target); // U SVIJETSKE KOORDINATNI SUS.
			target = jointsModel[type].InverseTransformDirection(target); // U KOORDINATNI SUS. ZGLOBA
			
			// Rotation to point vector of bone direction in direction of new position 
			Quaternion quat = Quaternion.FromToRotation(direction, target);
			
			// Rotation Application
			jointsModel[type].localRotation = jointsModel[type].localRotation  * quat;
			
		}
		*/
		
	}
	

	/**
	 *
	 * RotateJointWithAngle
	 *
	 */
	void RotateJointWithAngle (Vector3 target, Vector3 direction, JointType type) 
	{

		// Rotation estimation
		target = transform.TransformDirection(target); 					// Into the World coordinate system
		target = jointsModel[type].InverseTransformDirection(target); 	// Into the Joint coordinate system

		// Rotation to point vector of bone direction in direction of new position 
		Quaternion quat = Quaternion.FromToRotation(direction, target);

		// Rotation Application
		jointsModel[type].localRotation = jointsModel[type].localRotation  * quat;

	}


	/**
	 *
	 * jointPosition
	 * 
	 * Get filtered tracked joint position by joint index 
	 *
	 */
	Vector3 jointPosition (int jointIndex) 
	{
		return jointsModelFiltered[jointTypesArray[jointIndex]];
	}


	/**
	 *
	 * jointPosition
	 *
	 * Get filtered tracked joint position by joint type
	 */
	Vector3 jointPosition(JointType jt) 
	{
		return jointsModelFiltered[jt];
	}


	// SCALING SECTION

	/**
	 *
	 * scaleFiltering
	 *
	 * Filtering data needed for scaling with EWMA filter
	 *  
	 */
	void scaleFiltering(JointType type, bool bufferFilled) 
	{
		float xT = (typesToJoints[type].skeletonPoint.ImgCoordNormHorizontal)*2 - 1;
		float yT = (-1* typesToJoints[type].skeletonPoint.ImgCoordNormVertical)*2 + 1;
		float zT = DEPTH_CONSTANT;
		
		textureJointsPositionBufferS[type][jointsLastIndex[type]] = new Vector3 (xT, yT, zT);
		
		if (bufferFilled == true) 
		{
			var tmpTBuffer = textureJointsPositionBufferS[type];
			
			Vector3 tt = smoothVector(ref tmpTBuffer, windowSizeK);
			
			textureJointsFiltered[type] = tt;
			
			// Shift Buffer
			for (int i=0; i<windowSizeK; ++i) 
				textureJointsPositionBufferS[type][i] = textureJointsPositionBufferS[type][i+1];
		}
	}
	

	/**
	 * 
	 * calculateTranslationInZ
	 *
	 * Calculating translation in Z axis using model's first distance from camera 
	 * and scaling it (scale coeff is percentage of relative bone size change)
	 * in order to get current model's distance from camera
     * Using change in the relative bone size
     * scaletype defines which bone is relative bone for proximity measurement 
	 * 
	 */
	float calculateTranslationInZ (scaleType scaletype) 
	{
		float coeff;

		coeff = calculateScaleFiltered (scaletype);


		// 1 if it is the same, > 1 closer, < 1 smaller
		float currentDistanceFromCamera = firstDistanceFromCamera / coeff;
		
		// Debug.Log ("current Distance From Camera ="+currentDistanceFromCamera);
		float deltaDistance = currentDistanceFromCamera - firstDistanceFromCamera;

		// coordinate 0 is taken as referent point
		// because translation is estimated relatively anyway
		float zT =  DEPTH_CONSTANT - deltaDistance; 

		return zT;
	}


	/**
	 * 
	 * calculateTrackedSkltModelSkltRatio
	 *
	 * Calculating ratio between tracked skeleton and 3D model skeleton
     * Used for Z proximity demonstration
	 * 
	 */
	float calculateTrackedSkltModelSkltRatio (scaleType scaletype) 
	{
		Vector3 firstJointPosTr;
		Vector3 secondJointPosTr;
		Vector3 deltaTr;
		
		Vector3 firstJointPosModel;
		Vector3 secondJointPosModel;
		Vector3 deltaMo;

 		JointType firstJoint;
 		JointType secondJoint;
				
		switch (scaletype) 
		{
		case scaleType.WristElbow : 
				firstJoint = JointType.HandRight; 
				secondJoint = JointType.ElbowRight;
				firstJointPosModel =  mHandR.localPosition;
				secondJointPosModel = mElbowR.localPosition; 
			break;
		case scaleType.Torso : 
				firstJoint = JointType.Spine; 
				secondJoint = JointType.HipCenter;
				firstJointPosModel =  mSpine.localPosition;
				secondJointPosModel = mHipCenter.localPosition; 
			break;
		case scaleType.SpineShoulder : 
				firstJoint = JointType.Spine; 
				secondJoint = JointType.ShoulderRight; 
				firstJointPosModel =  mSpine.localPosition;
				secondJointPosModel = mShoulderR.localPosition; 
			break;
		case scaleType.HeadSpine : 
				firstJoint = JointType.Head; 
				secondJoint = JointType.Spine;
				firstJointPosModel =  mHead.localPosition;
				secondJointPosModel = mSpine.localPosition; 
			break;
		default : 
				firstJoint = JointType.Spine; 
				secondJoint = JointType.ShoulderRight; 
				firstJointPosModel =  mSpine.localPosition;
				secondJointPosModel = mShoulderR.localPosition; 
			break;

		}
		
		firstJointPosTr.x = textureXPos +     typesToJoints[firstJoint].skeletonPoint.ImgCoordNormHorizontal * textureDimensionX;
		firstJointPosTr.y = textureYPos + -1* typesToJoints[firstJoint].skeletonPoint.ImgCoordNormVertical * textureDimensionY;
		firstJointPosTr.z = DEPTH_CONSTANT;
		
		secondJointPosTr.x = textureXPos +     typesToJoints[secondJoint].skeletonPoint.ImgCoordNormHorizontal * textureDimensionX;
		secondJointPosTr.y = textureYPos + -1* typesToJoints[secondJoint].skeletonPoint.ImgCoordNormVertical * textureDimensionY;
		secondJointPosTr.z = DEPTH_CONSTANT;
		

		deltaTr = Camera.main.ScreenToWorldPoint(firstJointPosTr) - Camera.main.ScreenToWorldPoint(secondJointPosTr);
		deltaMo = firstJointPosModel - secondJointPosModel;


		return deltaTr.magnitude / deltaMo.magnitude;
	

	}


	/**
	 * 
	 * calculateScaleFiltered
	 *
	 * Scaling for translation in Z axis (non filtered method is not implemented yet)
     * 
	 */
	float calculateScaleFiltered (scaleType scaletype) 
	{
			
		float coeff = 1;
		JointType firstJoint;
 		JointType secondJoint;

		switch (scaletype) 
		{
		case scaleType.WristElbow : 
				firstJoint = JointType.HandRight; 
				secondJoint = JointType.ElbowRight;
			break;
		case scaleType.Torso : 
				firstJoint = JointType.Spine; 
				secondJoint = JointType.HipCenter;
			break;
		case scaleType.SpineShoulder : 
				firstJoint = JointType.Spine; 
				secondJoint = JointType.ShoulderRight;
			break;
		case scaleType.HeadSpine : 
				firstJoint = JointType.Head; 
				secondJoint = JointType.Spine; 
			break;
		default : 
				firstJoint = JointType.Spine; 
				secondJoint = JointType.ShoulderRight; 
			break;
		}

		
		Vector3 boneSize = textureJointsFiltered[firstJoint] - textureJointsFiltered[secondJoint];
		
		// Remember first size of relative bone for later comparison
		if (firstDataCapture) 
		{
			firstCaptureSizeOfRelativeBone = textureJointsFiltered[firstJoint] - textureJointsFiltered[secondJoint];
			// Wait for buffer to fill up, and smoothing vector gets value
			if (firstCaptureSizeOfRelativeBone != new Vector3(0,0,0)) 
			{
				++DataCaptureIndex;

				if (DataCaptureIndex > 4)
					firstDataCapture = false;
			}
			
		} 
		else // Later calculate coefficient
		{
			boneSize = textureJointsFiltered[firstJoint] - textureJointsFiltered[secondJoint];
			coeff = boneSize.magnitude / firstCaptureSizeOfRelativeBone.magnitude;
			// if coeff is < 0 relative bone is smaller, and user is further away
			// if coeff is > 0 relative bone is bigger, and user is closer to camera 
		}
		
		return coeff;

	}
	

}
