package application;


import javax.inject.Inject;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.HRCMotions.handGuiding;

import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.motionModel.HRCMotions;
import com.kuka.roboticsAPI.motionModel.HandGuidingMotion;
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
public class HandGuidingApp extends RoboticsAPIApplication {
	@Inject
	private LBR robot;
	private HandGuidingMotion hand;
	private final static String informationText = 
			        "Choose the type of Hand guiding." +"\n" +
					"\n" +"Note!"+ "\n" +
					"Please enable automatic mode and then run this program.";
	
	@Override
	public void initialize() {
		// initialize your application here
	}

	@Override
	public void run() {
		// your application execution starts here
		hand = new HandGuidingMotion();
		
		//hand.setJointLimitsMax(Math.toRadians(100),Math.toRadians(100),Math.toRadians(100),Math.toRadians(100),//	Math.toRadians(100),Math.toRadians(100));
		//hand.setJointLimitsMin(Math.toRadians(-100),Math.toRadians(-100),Math.toRadians(-100),Math.toRadians(-100),Math.toRadians(-100),Math.toRadians(-100));
		
		hand.setJointVelocityLimit(1.5);//rads/s
		hand.setCartVelocityLimit(50);//cm/s
		testMethod();
	}

	private void testMethod() {
		// TODO Auto-generated method stub
		int ret =0;
		while(ret != 3)
		{
			ret = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION,informationText,"HG","HGWVL","HRC","exit");
			switch (ret)
			{
			case 0:
				getLogger().info("Hand guide");
				robot.move(handGuiding());
				break;
			case 1:
				getLogger().info("Hand guiding with velocity limit");
				robot.move(hand);
				break;	
			case 2:
				getLogger().info("Human Robot Collaboration");
				robot.move(HRCMotions.handGuiding());
				break;
			case 3:
				getLogger().info("Finished");
				break;
			}  
			
		}
		
	}
}