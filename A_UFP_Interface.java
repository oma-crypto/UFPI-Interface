package application;


import java.util.Date;

import com.kuka.common.ThreadUtil;
import com.kuka.connectivity.motionModel.smartServo.ServoMotion;
import com.kuka.connectivity.motionModel.smartServoLIN.ISmartServoLINRuntime;
import com.kuka.connectivity.motionModel.smartServoLIN.SmartServoLIN;
import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.conditionModel.ConditionObserver;
import com.kuka.roboticsAPI.conditionModel.IAnyEdgeListener;
import com.kuka.roboticsAPI.conditionModel.ICondition;
import com.kuka.roboticsAPI.conditionModel.JointTorqueCondition;
import com.kuka.roboticsAPI.conditionModel.NotificationType;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.JointEnum;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.AbstractFrame;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.motionModel.HandGuidingMotion;
import com.kuka.roboticsAPI.motionModel.PTP;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.IMotionControlMode;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;


/**
 * Implementation of a robot application.
 * <p>
 * The application provides a {@link RoboticsAPITask#initialize()} and a 
 * {@link RoboticsAPITask#run()} method, which will be called successively in 
 * the application lifecycle. The application will terminate automatically after 
 * the {@link RoboticsAPITask#run()} method has finished or after stopping the 
 * task. The {@link RoboticsAPITask#dispose()} method will be called, even if an 
 * exception is thrown during initialization or run. 
 * <p>
 * <b>It is imperative to call <code>super.dispose()</code> when overriding the 
 * {@link RoboticsAPITask#dispose()} method.</b> 
 * 
 * @see #run()
 * @see #dispose()
 */
public class A_UFP_Interface extends RoboticsAPIApplication {
	private Controller kuka_Sunrise_Cabinet_1;
	private ISmartServoLINRuntime impedanceMode = null;
	private LBR lbr_iiwa_7_R800;
	private static final int stiffnessZ = 400; //in [N/m]
	private static final int stiffnessY = 400; //in [N/m]
	private static final int stiffnessX = 400; //in [N/m]
	private static final double[] MAX_TRANSLATION_VELOCITY = { 150, 150, 150 };
	private MediaFlangeIOGroup mFlangeTouch;
	private HRCEGP40 gripper;
	private final static String informationText = 
			"Recording motion application for move to several point as you need." +"\n" +
					"\n" + "Note!"+ "\n" + "Please enable automatic mode and then run this program.";
	
	private final static String informationTextax = 
			"Because you select a circular motion type; So at the first, go to the auxiliary point:";
	
	private final static String informationTextde = 
			"Now go to the destination point:"+"\n" +
					"\n" + "Note!"+ "\n" + "The point must be on a circular path, otherwise an error will occur when executing.";
	
	private final static String InformationText = 
			"Do you want to move? If yes, choose the type of motion for this point, else click stop:";
	
	private final static String InformationTextg = 
			"Do you want to Open or Closed the Gripper?";
	
	private final static String Informationtext = 
			"Do you want to re-program the recorded movements or terminate the program?";
	
	private final static String informationtext = 
			"Do you want to save this position?";
		
	private final static String InformationTextRe = 
			"Enter the number of re-executions for the Recorded movements from 1-12 times:";
	
	private final static String InformationTextch = 
			"Do you want to get to the point with the Hand Guiding Motion feature, or by adding a value to one of the " +
					"previous point axes (x, y, z), or selecting one of the previously designated points as reference points?";
	
	private final static String InformationTextch2 = 
			"To control the robot using the Jog keys, you need in the first to transition from automatic mode to T1 mode. " +
					"Subsequently, move the robot to your desired location, after that switch back to automatic mode and finally press the 'OK' button and run the interface to complete the program.";
	
	final static String InformationTextad = 
			"Select the axis that you want to add the value:";
	
	final static String InformationTextva = 
			"Enter the value of the offset:"+"\n" +
					"\n" + "Note!"+ "\n" + "The value must be within the workspace of the robot.";
	
	final static String InformationTextsi = 
			"Enter the value sign:";
	
	private double x = 0.05;
	private double y = 0.5;
	private int i = 0;
	private static int sampleIntervall=0;
	private JointPosition myJoints;
	private int[] myIndex = new int[2000000];
	private int[] mySave = new int[2000000];
	private String[] mycr = new String[2000000];
	private static int n = 0;
	private static int cr = 0;
	private static String[] TypeMO = new String[2000000];
	private Frame[] myPrintFrame = new Frame[2000000];
	private Frame[] auxiliaryFrame = new Frame[2000000];
	private ConditionObserver observing;
	private boolean BClash;
	private double MaxClashForce = 7.5;
	private double MinClashForce = -7.5;
	private int count = 0;
	private HandGuidingMotion Hand;
	JointPosition Home = new JointPosition(0,0,0,-Math.toRadians(90),0,Math.toRadians(90),0);
	
	static String InformationTextPo = 
			"Please input the number of the previously designated reference point:"+"\n" +
					"\n" + "Note!"+ "\n" + "If you input a number for a reference point that hasn't been established previously, the robot will move " +
					"to the last stored reference point. It's important to note that there are "+ n + " saved reference points.";
	
	 static String informationtextsa = 
			"Want to save this point as reference point number "+ (n+1) + " ?";
	
	@Override
	public void initialize() {
		kuka_Sunrise_Cabinet_1 = getController("KUKA_Sunrise_Cabinet_1");
		lbr_iiwa_7_R800 = (LBR) getDevice(kuka_Sunrise_Cabinet_1,
				"LBR_iiwa_7_R800_1");
		mFlangeTouch = new MediaFlangeIOGroup(kuka_Sunrise_Cabinet_1);
		gripper = new HRCEGP40(mFlangeTouch);
		this.setCollisionDetection();
		Hand = new HandGuidingMotion();
	}
	
	public void setCollisionDetection(){
		BClash = true;
		JointTorqueCondition jt1 = new JointTorqueCondition(lbr_iiwa_7_R800,JointEnum.J1,MinClashForce,MaxClashForce);
		JointTorqueCondition jt2 = new JointTorqueCondition(lbr_iiwa_7_R800,JointEnum.J2,MinClashForce,MaxClashForce);
		JointTorqueCondition jt3 = new JointTorqueCondition(lbr_iiwa_7_R800,JointEnum.J3,MinClashForce,MaxClashForce);
		JointTorqueCondition jt4 = new JointTorqueCondition(lbr_iiwa_7_R800,JointEnum.J4,MinClashForce,MaxClashForce);
		JointTorqueCondition jt5 = new JointTorqueCondition(lbr_iiwa_7_R800,JointEnum.J5,MinClashForce,MaxClashForce);
		JointTorqueCondition jt6 = new JointTorqueCondition(lbr_iiwa_7_R800,JointEnum.J6,MinClashForce,MaxClashForce);
		JointTorqueCondition jt7 = new JointTorqueCondition(lbr_iiwa_7_R800,JointEnum.J7,MinClashForce,MaxClashForce);
		ICondition collisionCondition = jt1.or(jt2, jt3, jt4, jt5, jt6, jt7);
		
		observing=getObserverManager().createConditionObserver(collisionCondition, NotificationType.OnEnable, new IAnyEdgeListener()
		{
			public void onAnyEdge(ConditionObserver conditionObserver, Date time, int missedEvents, boolean conditionValue){
				if(conditionValue){
					getLogger().info("force exceed");
					
						if(BClash){
							getLogger().info("Collision");
							getApplicationControl().setApplicationOverride(0.0d);
							mFlangeTouch.setLEDBlue(false);
							getLogger().info("Impact the robot with an external force for only once");
							ThreadUtil.milliSleep(1000);
							count = 0;
							BClash = !BClash;
						}
						else if(count == 1){
							mFlangeTouch.setLEDBlue(true);
							getLogger().info("Wait a little bit");
							ThreadUtil.milliSleep(1000);
							getApplicationControl().setApplicationOverride(1.0d);
							BClash = !BClash;
						}
					count++;}}
		 } );
    }
	
	@Override
	public void run() {
		Hand.setJointVelocityLimit(1.5);//rads/s
		Hand.setCartVelocityLimit(50);//cm/s
		sampleIntervall=0;
		n = 0;
		cr = 0;
		
		CartesianImpedanceControlMode impedanceControlMode = 	new CartesianImpedanceControlMode();
		impedanceControlMode.parametrize(CartDOF.X).setStiffness(stiffnessX);
		impedanceControlMode.parametrize(CartDOF.Y).setStiffness(stiffnessY);
		impedanceControlMode.parametrize(CartDOF.Z).setStiffness(stiffnessZ);
		impedanceControlMode.parametrize(CartDOF.ROT).setStiffness(50); //in[Nm/rad]. 
		
		int isCancel = getApplicationUI().displayModalDialog(ApplicationDialogType.INFORMATION, informationText, "OK", "Cancel");
        if (isCancel == 1)
        {
            return;
        }
		PTP ptpToHomePosition = ptp(Home);
		ptpToHomePosition.setJointVelocityRel(y).setJointAccelerationRel(x);
		lbr_iiwa_7_R800.move(ptpToHomePosition);
		gripper.setOpen();
		
		if (!ServoMotion.validateForImpedanceMode(lbr_iiwa_7_R800))
        {
            getLogger()
                    .info("Validation of torque model failed - correct your mass property settings");
            getLogger()
                    .info("Servo motion will be available for position controlled mode only, until validation is performed");
        }
		
		int run =2;
		while(run == 2)
		{
			if (i > 0)
	        {
				getLogger().info("Enter the positions that to be added:");
	        }

		int ret =0;
		while(ret != 4)
		{
			ret = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION,InformationText,"MovePTP","MovePTP_With_Impedance","MoveLIN","MoveCIRC","Stop");
			switch (ret)
			{
			case 0:
				TypeMO[sampleIntervall] = "PTP";
				RecordMotion();
				break;
			case 1:
				TypeMO[sampleIntervall] = "MovePTP_With_Impedance";
				RecordMotion();
				break;	
			case 2:
				TypeMO[sampleIntervall] = "LIN";
				RecordMotion();
				break;
			case 3:
				TypeMO[sampleIntervall] = "CIRC";
				RecordMotionCIRC();
				break;
			case 4:
				getLogger().info("Move away from the robot and reduce speed if it is high.");
				break;
			}  
		}
		observing.enable();
		run =1;
		while(run != 0 && run != 2)
		{
		int Re =getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION,InformationTextRe,"Once","Twice","3-Times","4-Times","5-Times","6-Times","7-Times","8-Times","9-Times","10-Times","11-Times","12-Times");	
		for(int c=0;c <= Re;c++)
		{
		mFlangeTouch.setLEDBlue(true);
		getLogger().info("Go to Home Position");
		getLogger().info("Execution No. " +(1+c));
		lbr_iiwa_7_R800.move(ptpToHomePosition);
		gripper.setOpen();
		ThreadUtil.milliSleep(1000);
		
		for(i=0;i<sampleIntervall;i++)
		{
			if(TypeMO[i] == "PTP"){
				getLogger().info("Go to point number " +(1+i) + " by using PTP Motion.");
				lbr_iiwa_7_R800.move(ptp(myPrintFrame[i]).setJointVelocityRel(y).setJointAccelerationRel(x));
			}
			else if (TypeMO[i] == "MovePTP_With_Impedance") {
				getLogger().info("Go to point number " +(1+i) + " by using Impedance Motion Mode.");
				observing.disable();
				runSmartServoLINMotion(impedanceControlMode);	
			}
			else if (TypeMO[i] == "LIN") {
				getLogger().info("Go to point number " +(1+i) + " by using LIN Motion.");
				lbr_iiwa_7_R800.move(lin(myPrintFrame[i]).setJointVelocityRel(y).setJointAccelerationRel(x));	
			}
			else{
				getLogger().info("Go to point number " +(1+i) + " by using CIRC Motion.");
				lbr_iiwa_7_R800.move(circ(auxiliaryFrame[i], myPrintFrame[i]).setJointVelocityRel(y).setJointAccelerationRel(x));
			}	
			
			if(myIndex[i]!= i){
				if(gripper.getStatus()[0]){
					ThreadUtil.milliSleep(500);
					gripper.setClose();
					getLogger().info("Gripper is close.");
					ThreadUtil.milliSleep(500);
				}
				else{
					ThreadUtil.milliSleep(500);
					gripper.setOpen();
					getLogger().info("Gripper is open.");
					ThreadUtil.milliSleep(500);
				}	
			}
			observing.enable();
		 }
	   }
		getLogger().info("Finished");
		mFlangeTouch.setLEDBlue(false);
		run = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION,Informationtext,"Terminate","Re-program","Re-program_with_adding_points");
		  }
		}
		mFlangeTouch.setLEDBlue(true);
		getLogger().info("Back to Home Position");
		lbr_iiwa_7_R800.move(ptpToHomePosition);
		gripper.setOpen();
		for(int c=0;c<=10;c++)
		{
			mFlangeTouch.setLEDBlue(true);
			ThreadUtil.milliSleep(250);
			mFlangeTouch.setLEDBlue(false);
			ThreadUtil.milliSleep(250);
		}
	}
	
	private void RecordMotion() {
		getLogger().info("Go to point number " +(1+sampleIntervall) + " :");
		ApproachToReach();
		
		mFlangeTouch.setLEDBlue(true);
		myIndex[sampleIntervall] = sampleIntervall;
		int grip = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION,InformationTextg,"No","Yes");
		switch (grip)
		{
		case 0:
			break;
		case 1:
			myIndex[sampleIntervall] = sampleIntervall+1;	
			if(gripper.getStatus()[0]){
				gripper.setClose();
				getLogger().info("Gripper is close.");
			}
			else{
				gripper.setOpen();
				getLogger().info("Gripper is open.");
			}	
			break;	
		}  
		mFlangeTouch.setLEDBlue(false);
		
		myJoints = lbr_iiwa_7_R800.getCurrentJointPosition();							
		myPrintFrame[sampleIntervall] = lbr_iiwa_7_R800.getForwardKinematic(myJoints).copyWithRedundancy();
		sampleIntervall++;
	}
	
	private void RecordMotionCIRC() {
		getLogger().info("Go to point number " +(1+sampleIntervall) + " :");
	//	getLogger().info("Because you select a circular motion type; So at the first, go to the auxiliary point:");
		getApplicationUI().displayModalDialog(ApplicationDialogType.INFORMATION, informationTextax, "OK");
		cr =1;
		ApproachToReach();

		myJoints = lbr_iiwa_7_R800.getCurrentJointPosition();							
		auxiliaryFrame[sampleIntervall] = lbr_iiwa_7_R800.getForwardKinematic(myJoints).copyWithRedundancy();
		cr = 2;
	//	getLogger().info("Now go to the destination point:");
		getApplicationUI().displayModalDialog(ApplicationDialogType.INFORMATION, informationTextde, "OK");
		ApproachToReach();
		
		cr = 0;
		mFlangeTouch.setLEDBlue(true);
		myIndex[sampleIntervall] = sampleIntervall;
		int grip = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION,InformationTextg,"No","Yes");
		switch (grip)
		{
		case 0:
			break;
		case 1:
			myIndex[sampleIntervall] = sampleIntervall+1;	
			if(gripper.getStatus()[0]){
				gripper.setClose();
				getLogger().info("Gripper is close.");
			}
			else{
				gripper.setOpen();
				getLogger().info("Gripper is open.");
			}	
			break;	
		}  
		mFlangeTouch.setLEDBlue(false);
		
		myJoints = lbr_iiwa_7_R800.getCurrentJointPosition();							
		myPrintFrame[sampleIntervall] = lbr_iiwa_7_R800.getForwardKinematic(myJoints).copyWithRedundancy();
		sampleIntervall++;
	}
	
	protected void runSmartServoLINMotion(final IMotionControlMode controlMode)
    {
		getLogger().info("Hold position in impedance control mode");
        AbstractFrame initialPosition = lbr_iiwa_7_R800.getCurrentCartesianPosition(lbr_iiwa_7_R800.getFlange());
        // Create a new smart servo linear motion
        SmartServoLIN SmartMotion = new SmartServoLIN(initialPosition);

        SmartMotion.setMaxTranslationVelocity(MAX_TRANSLATION_VELOCITY);
        SmartMotion.setMinimumTrajectoryExecutionTime(20e-3);

        //getLogger().info("Starting the SmartServoLIN in " + controlMode);
		lbr_iiwa_7_R800.moveAsync(SmartMotion.setMode(controlMode));

        //getLogger().info("Get the runtime of the SmartServoLIN motion");
        impedanceMode = SmartMotion.getRuntime();
 		
	            	ThreadUtil.milliSleep(500);
	                // Update the smart servo LIN runtime
	                // theSmartServoLINRuntime.updateWithRealtimeSystem();
	                // Set new destination
	                impedanceMode.setDestination(myPrintFrame[i]);
	                ThreadUtil.milliSleep(6000);
	                impedanceMode.setDestination(myPrintFrame[i]);
	                getLogger().info("Turn off the impedance control mode");
	                impedanceMode.stopMotion();
	    }
	
	private void ApproachToReach() {
		informationtextsa = 
				"Want to save this point as reference point number "+ (n+1) + " ?";
		
		InformationTextPo = 
				"Please input the number of the previously designated reference point:"+"\n" +
						"\n" + "Note!"+ "\n" + "If you input a number for a reference point that hasn't been established previously, the robot will move " +
						"to the last stored reference point. It's important to note that there are "+ n + " saved reference points.";
		int ch = 0;
		if(sampleIntervall!= 0){
		int sav =0;
		while(sav != 1)
		{
			if(ch == 2 || ch == 3){
				if (cr == 2){
					lbr_iiwa_7_R800.move(lin(auxiliaryFrame[sampleIntervall]).setJointVelocityRel(y).setJointAccelerationRel(x));
				}
				else{
					lbr_iiwa_7_R800.move(lin(myPrintFrame[sampleIntervall-1]).setJointVelocityRel(y).setJointAccelerationRel(x));
				}
			}
		 ch =getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION,InformationTextch,"Hand Guiding","Manual Keys","Adding_Value_on_Axes","Reference Points");
		switch (ch)
			{
				case 0:
					getLogger().info("Go to this point by using hand guiding.");
					lbr_iiwa_7_R800.move(Hand);
				break;
				case 1:
					getLogger().info("Go to this point by using manual keys.");
					getApplicationUI().displayModalDialog(ApplicationDialogType.INFORMATION,InformationTextch2,"OK");
					myJoints = lbr_iiwa_7_R800.getCurrentJointPosition();							
					myPrintFrame[sampleIntervall] = lbr_iiwa_7_R800.getForwardKinematic(myJoints).copyWithRedundancy();
					lbr_iiwa_7_R800.move(lin(myPrintFrame[sampleIntervall]).setJointVelocityRel(y).setJointAccelerationRel(x));
				break;
				case 2:
					int ad =getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION,InformationTextad,"X_Axis","Y_Axes","Z_Axis");
					if (cr == 2){
						myPrintFrame[sampleIntervall] = auxiliaryFrame[sampleIntervall].copyWithRedundancy();
					}
					else{
						myPrintFrame[sampleIntervall] = myPrintFrame[sampleIntervall-1].copyWithRedundancy();
					}
					int va =getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION,InformationTextva,"1mm","3mm","5mm","10mm","20mm","30mm","40mm","50mm","100mm","200mm","300mm","500mm");
					switch (va)
					{
					case 0:
						va=1;
						break;
					case 1:
						va=3;
						break;	
					case 2:
						va=5;
						break;
					case 3:
						va=10;
						break;
					case 4:
						va=20;
						break;
					case 5:
						va=30;
						break;
					case 6:
						va=40;
						break;	
					case 7:
						va=50;
						break;
					case 8:
						va=100;
						break;
					case 9:
						va=200;
						break;	
					case 10:
						va=300;
						break;
					case 11:
						va=500;
						break;
					}  
					int si =getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION,InformationTextsi,"(-)","(+)");
					switch (si)
					{
					case 0:
						va=-va;
						break;
					case 1:
						break;
					} 
					switch (ad)
					{
					case 0:
						getLogger().info("Go to this point by adding value on X Axis.");
						myPrintFrame[sampleIntervall].setX(myPrintFrame[sampleIntervall].getX()+ va);
						lbr_iiwa_7_R800.move(lin(myPrintFrame[sampleIntervall]).setJointVelocityRel(y).setJointAccelerationRel(x));
						break;
					case 1:
						getLogger().info("Go to this point by adding value on Y Axis.");
						myPrintFrame[sampleIntervall].setY(myPrintFrame[sampleIntervall].getY()+ va);
						lbr_iiwa_7_R800.move(lin(myPrintFrame[sampleIntervall]).setJointVelocityRel(y).setJointAccelerationRel(x));
						break;	
					case 2:
						getLogger().info("Go to this point by adding value on Z Axis.");
						myPrintFrame[sampleIntervall].setZ(myPrintFrame[sampleIntervall].getZ()+ va);
						lbr_iiwa_7_R800.move(lin(myPrintFrame[sampleIntervall]).setJointVelocityRel(y).setJointAccelerationRel(x));
						break;
					}
					
				break;
				case 3:
					int Po =getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION,InformationTextPo,"1","2","3","4","5","6","7","8","9","10","11","12");
					if(Po >= n){
						Po = n;
					}
					getLogger().info("Go to this point as point number "+(1+Po)+ " that previously saved.");
					if (mycr[Po] == "auxiliary"){
						lbr_iiwa_7_R800.move(lin(auxiliaryFrame[mySave[Po]]).setJointVelocityRel(y).setJointAccelerationRel(x));
					}
					else{
					lbr_iiwa_7_R800.move(lin(myPrintFrame[mySave[Po]]).setJointVelocityRel(y).setJointAccelerationRel(x));
					}
				break;
			}
		sav = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION,informationtext,"No","Yes");
		}
	  }
		else{
			int sav1 =0;
			while(sav1 != 1)
			{
			int ch2 =getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION,InformationTextch,"Hand Guiding","Manual Keys");
				switch (ch2)
					{
						case 0:
							getLogger().info("Go to this point by using hand guiding.");
							lbr_iiwa_7_R800.move(Hand);
						break;
						case 1:
							getLogger().info("Go to this point by using manual keys.");
							getApplicationUI().displayModalDialog(ApplicationDialogType.INFORMATION,InformationTextch2,"OK");
							myJoints = lbr_iiwa_7_R800.getCurrentJointPosition();							
							myPrintFrame[sampleIntervall] = lbr_iiwa_7_R800.getForwardKinematic(myJoints).copyWithRedundancy();
							lbr_iiwa_7_R800.move(lin(myPrintFrame[sampleIntervall]).setJointVelocityRel(y).setJointAccelerationRel(x));
						break;
					}
				sav1 = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION,informationtext,"No","Yes");
		  }
		}
		if (n < 12 && ch != 3){
			int sav2 = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION,informationtextsa,"No","Yes");
			if (sav2 == 1)
			{
				if (cr == 1){
					mycr[n] = "auxiliary";
				}
				else{
					mycr[n] = "Print";
					}
				mySave[n] = sampleIntervall;
				n++;
			}
		}
	}
}