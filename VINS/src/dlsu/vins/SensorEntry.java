/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package dlsu.vins;

import java.util.LinkedList;

/**
 *
 * @author CCS
 */
public class SensorEntry {

    private int entryNumber;
    private int second;
    private String date;
    private double acc_x;
    private double acc_y;
    private double acc_z;
    private double gyro_x;
    private double gyro_y;
    private double gyro_z;
    private double orient_x;
    private double orient_y;
    private double orient_z;
    private double mag_x;
    private double mag_y;
    private double mag_z;
    private double acc_norm;
    private String remarks;
    private LinkedList<Double> sensors;
    private long timeRecorded;

    public LinkedList<Double> getSensors() {
        return sensors;
    }

    public void setSensors(LinkedList<Double> sensors) {
        this.sensors = sensors;
    }

    public int getEntryNumber() {
        return entryNumber;
    }

    public void setEntryNumber(int entryNumber) {
        this.entryNumber = entryNumber;
    }
    
    public long getTimeRecorded(){
    	return timeRecorded;
    }
    
    public void setTimeRecorded(long time){
    	this.timeRecorded = time;
    }
    
    // not called
    public void buildSensorList()
    {
        sensors = new LinkedList<Double>();
        sensors.add(acc_x);    // 0
        sensors.add(acc_y);    // 1
        sensors.add(acc_z);    // 2
        sensors.add(gyro_x);   // 3
        sensors.add(gyro_y);   // 4
        sensors.add(gyro_z);   // 5
    }
    
    public double getAcc_x() {
        return acc_x;
    }

    public void setAcc_x(double acc_x) {
        this.acc_x = acc_x;
    }

    public double getAcc_y() {
        return acc_y;
    }

    public void setAcc_y(double acc_y) {
        this.acc_y = acc_y;
    }

    public double getAcc_z() {
        return acc_z;
    }

    public void setAcc_z(double acc_z) {
        this.acc_z = acc_z;
    }

    public String getDate() {
        return date;
    }

    public void setDate(String date) {
        this.date = date;
    }

    public double getGyro_x() {
        return gyro_x;
    }

    public void setGyro_x(double gyro_x) {
        this.gyro_x = gyro_x;
    }

    public double getGyro_y() {
        return gyro_y;
    }

    public void setGyro_y(double gyro_y) {
        this.gyro_y = gyro_y;
    }

    public double getGyro_z() {
        return gyro_z;
    }

    public void setGyro_z(double gyro_z) {
        this.gyro_z = gyro_z;
    }

    public double getMag_x() {
        return mag_x;
    }

    public void setMag_x(double mag_x) {
        this.mag_x = mag_x;
    }

    public double getMag_y() {
        return mag_y;
    }

    public void setMag_y(double mag_y) {
        this.mag_y = mag_y;
    }

    public double getMag_z() {
        return mag_z;
    }

    public void setMag_z(double mag_z) {
        this.mag_z = mag_z;
    }

    public double getOrient_x() {
        return orient_x;
    }

    public void setOrient_x(double orient_x) {
        this.orient_x = orient_x;
    }

    public double getOrient_y() {
        return orient_y;
    }

    public void setOrient_y(double orient_y) {
        this.orient_y = orient_y;
    }

    public double getOrient_z() {
        return orient_z;
    }

    public void setOrient_z(double orient_z) {
        this.orient_z = orient_z;
    }

    public String getRemarks() {
        return remarks;
    }

    public void setRemarks(String remarks) {
        this.remarks = remarks;
    }

    public int getSecond() {
        return second;
    }

    public void setSecond(int second) {
        this.second = second;
    }
    
    public double getAcc_norm() {
        return this.calculateAcc_norm();
    	//return acc_norm;
    }

    public void setAcc_norm(double acc_norm) {
        this.acc_norm = acc_norm;
    }
    
    public double calculateAcc_norm(){
       return Math.sqrt(Math.pow(acc_x,2)+Math.pow(acc_y,2)+Math.pow(acc_z,2));
    }
    
    public double calculateGyro_norm(){
        return Math.sqrt(Math.pow(gyro_x,2)+Math.pow(gyro_y,2)+Math.pow(gyro_z,2));
    }
    
    public String toRawString(){
    	return getAcc_x()+","+getAcc_y()+","+getAcc_z()+","+getGyro_x()+","+getGyro_y()+","+getGyro_z()+","+getOrient_x()+","+getOrient_y()+","+getOrient_z();
    }
    

}
