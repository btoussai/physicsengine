package cataclysm.record;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.RandomAccessFile;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Date;

import cataclysm.PhysicsWorld;
import cataclysm.broadphase.staticmeshes.StaticMesh;
import cataclysm.wrappers.RigidBody;

/**
 * This class enables to record the state of a physics world into a file. All
 * simulated frames will be written in the file until the record is stopped. The
 * record can be played again in another physics simulation through a
 * {@link PhysicsPlayer}
 * 
 * @author Briac
 *
 */
public class PhysicsRecorder {

	private final RecordFile file;
	private final Frame currentFrame;
	private int totalFrameCount = 0;
	private long fileSize = 0;

	/**
	 * The position of the int encoding totalFrameCount in the file
	 */
	private int frameCountPosition = 0;

	/**
	 * Records the state of the world in a file
	 * 
	 * @param path               the destination file
	 * @param currentElapsedTime the time that has passed in the simulation since
	 *                           its creation.
	 * @throws IOException 
	 */
	public PhysicsRecorder(String path, double currentElapsedTime) throws IOException {
		this.file = new RecordFile(path, false);
		currentFrame = new Frame();

		writeDate();

		// we write two 0, the actual values will be written when the flushAndClose
		// method
		// will be called
		writeTotalFrameCount();

		fileSize = file.getPosition();
	}

	private void writeDate() {
		DateFormat df = new SimpleDateFormat("MM/dd/yyyy HH:mm:ss");
		Date today = Calendar.getInstance().getTime();
		String reportDate = df.format(today);
		file.writeLine(reportDate);
		frameCountPosition = file.getPosition();
	}

	private void writeTotalFrameCount() {
		file.writeInt(totalFrameCount);
	}

	public void newFrame() {
		totalFrameCount++;
		currentFrame.reset();
	}

	public Frame getCurrentFrame() {
		return currentFrame;
	}

	public void endOfFrame() {
		currentFrame.write(file);
		fileSize = file.getPosition();
	}

	/**
	 * Stops the recording and closes all in/out streams.
	 * @param physicsWorld 
	 */
	public void close(PhysicsWorld physicsWorld) {
		
		//we consider that all bodies in the recording get deleted at the end of it
		//so that we know what objects are present if we want to replay starting from the end.
		newFrame();
		currentFrame.fillBodies(new ArrayList<RigidBody>(), physicsWorld.getBodies());
		currentFrame.fillMeshes(new ArrayList<StaticMesh>(), physicsWorld.getMeshes());
		endOfFrame();
		
		fileSize = file.flushAndClose();
		
		

		System.out.println("Closing PhysicsRecord: total frames: " + totalFrameCount + " totalSize: " + fileSize);

		File f = new File(file.getFilePath());
		try {
			RandomAccessFile rf = new RandomAccessFile(f, "rw");
			rf.skipBytes(frameCountPosition);
			rf.writeInt(totalFrameCount);
			rf.close();
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}

	}

}
