package application;


import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import java.util.concurrent.TimeUnit;

import javax.inject.Inject;

import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
import com.kuka.roboticsAPI.applicationModel.tasks.CycleBehavior;
import com.kuka.roboticsAPI.applicationModel.tasks.RoboticsAPICyclicBackgroundTask;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.motionModel.PTP;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKey;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyBar;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyListener;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyAlignment;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyEvent;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyLED;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyLEDSize;


/**
 * Implementation of a cyclic background task.
 * <p>
 * It provides the {@link RoboticsAPICyclicBackgroundTask#runCyclic} method 
 * which will be called cyclically with the specified period.<br>
 * Cycle period and initial delay can be set by calling 
 * {@link RoboticsAPICyclicBackgroundTask#initializeCyclic} method in the 
 * {@link RoboticsAPIBackgroundTask#initialize()} method of the inheriting 
 * class.<br>
 * The cyclic background task can be terminated via 
 * {@link RoboticsAPICyclicBackgroundTask#getCyclicFuture()#cancel()} method or 
 * stopping of the task.
 * @see UseRoboticsAPIContext
 * 
 */
public class BackgroundGripper extends RoboticsAPICyclicBackgroundTask {
	
	@Inject
	private LBR lbr_iiwa_7_R800;
	
	private UserKeyLED KeyLED = UserKeyLED.Yellow;
	private Controller kuka_Sunrise_Cabinet_1;
	private MediaFlangeIOGroup mFlangeTouch;
	private HRCEGP40 gripper;
	private String value = " ";
	JointPosition Home = new JointPosition(0,0,0,-Math.toRadians(90),0,0,0);
	PTP ptpToHomePosition = ptp(Home);
	
	
	public void initialize() {
		kuka_Sunrise_Cabinet_1 = getController("KUKA_Sunrise_Cabinet_1");
		initializeCyclic(0, 50, TimeUnit.MILLISECONDS,
				CycleBehavior.BestEffort);
		
		mFlangeTouch = new MediaFlangeIOGroup(kuka_Sunrise_Cabinet_1);
		gripper = new HRCEGP40(mFlangeTouch);
		ptpToHomePosition.setJointVelocityRel(0.25);
		
		IUserKeyBar gripperBar = getApplicationUI().createUserKeyBar("Gripper");
		IUserKeyListener listener1 = new IUserKeyListener() {
			@Override
			public void onKeyEvent(IUserKey key, UserKeyEvent event) {
				if (event == UserKeyEvent.FirstKeyDown) {
					gripper.setClose();
					value = "Close";
				 }
				 else if (event == UserKeyEvent.SecondKeyDown) {
				 gripper.setOpen();
				 value = "Open";
				 }
				key.setText(UserKeyAlignment.Middle, value);
			}
			};
					
			final IUserKey GripperKey1 = gripperBar.addDoubleUserKey(0, listener1, false);
			GripperKey1.setText(UserKeyAlignment.TopMiddle, "grip");
			GripperKey1.setText(UserKeyAlignment.Middle,value);
			GripperKey1.setText(UserKeyAlignment.BottomMiddle, "release");	
			
					IUserKeyListener listener2 = new IUserKeyListener() {
						@Override
						public void onKeyEvent(IUserKey key, UserKeyEvent event) {
							if (event == UserKeyEvent.FirstKeyDown) {
								if(gripper.getStatus()[0]){
									gripper.setClose();
									value = "Close";
								}
								else{
									gripper.setOpen();
									value = "Open";
								}
							 }
							 else if (event == UserKeyEvent.SecondKeyDown) {
								 if(mFlangeTouch.getLEDBlue()== true){
									 mFlangeTouch.setLEDBlue(false);
									 KeyLED = UserKeyLED.Red;
									}
									else{
									mFlangeTouch.setLEDBlue(true);
									KeyLED = UserKeyLED.Green;
									    } 
								 }
							GripperKey1.setText(UserKeyAlignment.Middle,value);
							key.setText(UserKeyAlignment.Middle, "Omar");
							key.setLED(UserKeyAlignment.BottomRight, KeyLED,UserKeyLEDSize.Normal);
								}
								};
								
					IUserKey GripperKey2 = gripperBar.addDoubleUserKey(2, listener2, true);
					GripperKey2.setText(UserKeyAlignment.TopMiddle, "obverse");
					GripperKey2.setText(UserKeyAlignment.Middle,"Omar");
					GripperKey2.setText(UserKeyAlignment.BottomLeft, "LED");
					GripperKey2.setLED(UserKeyAlignment.BottomRight, KeyLED,UserKeyLEDSize.Normal);
					gripperBar.publish();
	}
	
	
	@Override
	public void runCyclic() {
		if(mFlangeTouch.getUserButton()){
			getLogger().info("Move to home position");
			lbr_iiwa_7_R800.move(ptpToHomePosition);
		}
	}
}
