package application;


import java.util.Date;

import javax.inject.Inject;
import com.kuka.common.ThreadUtil;
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
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.motionModel.PTP;
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
 * @see UseRoboticsAPIContext
 * @see #initialize()
 * @see #run()
 * @see #dispose()
 */
public class PickAndPlace extends RoboticsAPIApplication {
	private HRCEGP40 Gripper;
	private MediaFlangeIOGroup mFlangeTouch;
	private Controller kuka_Sunrise_Cabinet_1;
	private ConditionObserver observing;
	private boolean BClash;
	private double MaxClashForce = 7.5;
	private double MinClashForce = -7.5;
	private double x = 0.1;
	@Inject
	private LBR lBR_Omar;
	private final static String informationText = "Cubes have been moved. If you want to return it? So click on Yes.";
	private int count = 0;
	
	@Override
	public void initialize() {
		//Attach gripper to flange as TCP.
		kuka_Sunrise_Cabinet_1 = getController("KUKA_Sunrise_Cabinet_1");
		lBR_Omar = (LBR) getDevice(kuka_Sunrise_Cabinet_1,
				"LBR_iiwa_7_R800_1");
		mFlangeTouch = new MediaFlangeIOGroup(kuka_Sunrise_Cabinet_1);
		Gripper = new HRCEGP40(mFlangeTouch);
		this.setCollisionDetection();
		observing.enable();
	}
	public void setCollisionDetection(){
		BClash = true;
		JointTorqueCondition jt1 = new JointTorqueCondition(lBR_Omar,JointEnum.J1,MinClashForce,MaxClashForce);
		JointTorqueCondition jt2 = new JointTorqueCondition(lBR_Omar,JointEnum.J2,MinClashForce,MaxClashForce);
		JointTorqueCondition jt3 = new JointTorqueCondition(lBR_Omar,JointEnum.J3,MinClashForce,MaxClashForce);
		JointTorqueCondition jt4 = new JointTorqueCondition(lBR_Omar,JointEnum.J4,MinClashForce,MaxClashForce);
		JointTorqueCondition jt5 = new JointTorqueCondition(lBR_Omar,JointEnum.J5,MinClashForce,MaxClashForce);
		JointTorqueCondition jt6 = new JointTorqueCondition(lBR_Omar,JointEnum.J6,MinClashForce,MaxClashForce);
		JointTorqueCondition jt7 = new JointTorqueCondition(lBR_Omar,JointEnum.J7,MinClashForce,MaxClashForce);
		ICondition collisionCondition = jt1.or(jt2, jt3, jt4, jt5, jt6, jt7);
		
		observing=getObserverManager().createConditionObserver(collisionCondition, NotificationType.OnEnable, new IAnyEdgeListener()
		{
			public void onAnyEdge(ConditionObserver conditionObserver, Date time, int missedEvents, boolean conditionValue){
				if(conditionValue){
					getLogger().info("force exceed");
					
						if(BClash){
							getLogger().info("Collision");
							getApplicationControl().setApplicationOverride(0.0d);
							mFlangeTouch.setLEDBlue(true);
							getLogger().info("Impact the robot with an external force for only once");
							ThreadUtil.milliSleep(1000);
							count = 0;
							BClash = !BClash;
						}
						else if(count == 1){
							mFlangeTouch.setLEDBlue(false);
							getLogger().info("Wait a little bit");
							ThreadUtil.milliSleep(1000);
							getApplicationControl().setApplicationOverride(1.0d);
							BClash = !BClash;
						}
					count++;}}
		 }
           );
    }

	@Override
	public void run() {
		// your application execution starts here
		getLogger().info("Pick & Place application for moving cubes");
		Gripper.setOpen();
		Frame omar= new Frame(645,220,510,Math.PI,0,Math.PI);
		Frame P1 = omar.copyWithRedundancy();;
		Frame P2 = P1.copyWithRedundancy();
		P2.setZ(P2.getZ()- 103.1);
		Frame P3 = P1.copyWithRedundancy();
		P3.setY(P3.getY()- 447.2);
		Frame P4 = P3.copyWithRedundancy();
		P4.setZ(P4.getZ()- 243.7);
		
		getLogger().info("Go to Home Position");
		 JointPosition Home = new JointPosition(0,0,0,-Math.toRadians(90),0,Math.toRadians(90),0);
			PTP ptpToHomePosition = ptp(Home);
			ptpToHomePosition.setJointVelocityRel(0.25).setJointAccelerationRel(x);
			lBR_Omar.move(ptpToHomePosition);
		
		getLogger().info("Go to move cube number " +1);
		lBR_Omar.move(ptp(P1).setJointAccelerationRel(x));
		lBR_Omar.move(lin(P2).setJointAccelerationRel(x));
		ThreadUtil.milliSleep(100);
		Gripper.setClose();
		ThreadUtil.milliSleep(100);
		lBR_Omar.move(lin(P1).setJointAccelerationRel(0.05));
		lBR_Omar.move(ptp(P3).setJointAccelerationRel(x));
		lBR_Omar.move(lin(P4).setJointAccelerationRel(x));
		ThreadUtil.milliSleep(100);
		Gripper.setOpen();
		ThreadUtil.milliSleep(100);
		lBR_Omar.move(lin(P3).setJointAccelerationRel(0.05));
		
		for(int i=1;i<=5;i++)
		{
			getLogger().info("Go to move cube number " +(1+i));
			lBR_Omar.move(ptp(P1).setJointAccelerationRel(x));
			lBR_Omar.move(lin(P2.setZ(P2.getZ()- 28)).setJointAccelerationRel(x));
			ThreadUtil.milliSleep(100);
			Gripper.setClose();
			ThreadUtil.milliSleep(100);
			lBR_Omar.move(lin(P1).setJointAccelerationRel(0.05));
			lBR_Omar.move(ptp(P3).setJointAccelerationRel(x));
			lBR_Omar.move(lin(P4.setZ(P4.getZ()+ 28)).setJointAccelerationRel(x));
			ThreadUtil.milliSleep(100);
			Gripper.setOpen();
			ThreadUtil.milliSleep(100);
			lBR_Omar.move(lin(P3).setJointAccelerationRel(0.05));
		}
		getLogger().info("Back to Home Position");
		lBR_Omar.move(ptpToHomePosition);
		
		getLogger().info("Show modal dialog and wait for user to confirm");
        int rev = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, informationText, "Yes", "No");
        if (rev == 0)
        {
        	getLogger().info("Return the cubes to the first position");
    		Gripper.setOpen();
    		
    		getLogger().info("Go to return cube number " +1);
    		lBR_Omar.move(ptp(P3).setJointAccelerationRel(x));
    		lBR_Omar.move(lin(P4).setJointAccelerationRel(x));
    		ThreadUtil.milliSleep(100);
    		Gripper.setClose();
    		ThreadUtil.milliSleep(100);
    		lBR_Omar.move(lin(P3).setJointAccelerationRel(0.05));
    		lBR_Omar.move(ptp(P1).setJointAccelerationRel(x));
    		lBR_Omar.move(lin(P2).setJointAccelerationRel(x));
    		ThreadUtil.milliSleep(100);
    		Gripper.setOpen();
    		ThreadUtil.milliSleep(100);
    		lBR_Omar.move(lin(P1).setJointAccelerationRel(0.05));
    		
    		
    		for(int i=1;i<=5;i++)
    		{
    			getLogger().info("Go to return the cube number " +(1+i));
    			lBR_Omar.move(ptp(P3).setJointAccelerationRel(x));
    			lBR_Omar.move(lin(P4.setZ(P4.getZ()- 28)).setJointAccelerationRel(x));
    			ThreadUtil.milliSleep(100);
    			Gripper.setClose();
    			ThreadUtil.milliSleep(100);
    			lBR_Omar.move(lin(P3).setJointAccelerationRel(0.05));
    			lBR_Omar.move(ptp(P1).setJointAccelerationRel(x));
    			lBR_Omar.move(lin(P2.setZ(P2.getZ()+ 28)).setJointAccelerationRel(x));
    			ThreadUtil.milliSleep(100);
    			Gripper.setOpen();
    			ThreadUtil.milliSleep(100);
    			lBR_Omar.move(lin(P1).setJointAccelerationRel(0.05));
    		}
    		getLogger().info("Back to Home Position");
    		lBR_Omar.move(ptpToHomePosition);
    		
    		getLogger().info("Cubes have been return");
        }
		
		getLogger().info("Finished");
		for(int c=0;c<=10;c++)
		{
			mFlangeTouch.setLEDBlue(true);
			ThreadUtil.milliSleep(250);
			mFlangeTouch.setLEDBlue(false);
			ThreadUtil.milliSleep(250);
		}
	}
}