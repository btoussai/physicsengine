package cataclysm.record;

import java.io.FileNotFoundException;

import cataclysm.PhysicsWorld;

public class Recorder {
	
	private final PhysicsWorld world;
	private final Record record;
	private float timeStamp = 0;
	
	public Recorder(PhysicsWorld world, String outPath) throws FileNotFoundException {
		this.world = world;
		this.record = new Record(outPath, false);
	}
	
	
	
	public void startRecord() {
		timeStamp = 0;
		
		Frame f0 = new Frame(world);
		
	}
	
	public void startPlayback() {
		
	}

	public void recordFrame(float timeStep) {
		timeStamp += timeStep;
	}
	
	
	
}
