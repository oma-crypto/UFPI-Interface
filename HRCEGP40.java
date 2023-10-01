package application;

import com.kuka.generated.ioAccess.MediaFlangeIOGroup;

public class HRCEGP40 {
	
	private MediaFlangeIOGroup mediaFlange;	

	//Constructor
	public HRCEGP40(MediaFlangeIOGroup flange){
		
		this.mediaFlange = flange;	
	}
	//Set the Gripper open
	public void setOpen(){
		this.mediaFlange.setOutputX3Pin1(true);
		this.mediaFlange.setOutputX3Pin11(false);
	}
	//Set the Gripper closed
	public void setClose(){
		this.mediaFlange.setOutputX3Pin1(false);
		this.mediaFlange.setOutputX3Pin11(true);
	}
	//Get Inductive Sensor Information
	public boolean[] getStatus(){
		
		boolean status[] = {this.mediaFlange.getInputX3Pin10(), this.mediaFlange.getInputX3Pin16()};		
		return status;
	}
	//Setter to Power down the X3 Interface
	public void setPowerOff(){
		this.mediaFlange.setSwitchOffX3Voltage(false);

	}
	//Setter Powering on the X3 Interface
	public void setPowerOn(){
		this.mediaFlange.setSwitchOffX3Voltage(false);
	}
	//Getter for the Power Switch
	public boolean getPower(){
		
		return this.mediaFlange.getSwitchOffX3Voltage();
	}   
	//Method resetting possible Errors
	public void reset(){
		this.mediaFlange.setOutputX3Pin1(true);
		this.mediaFlange.setOutputX3Pin11(true);		
	}
	
}
