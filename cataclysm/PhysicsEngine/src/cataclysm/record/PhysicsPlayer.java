package cataclysm.record;

import java.io.FileNotFoundException;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Date;

/**
 * This class enables to play back a record of a simulation.
 * 
 * @see PhysicsRecorder
 * 
 * @author Briac
 *
 */
public class PhysicsPlayer {

	private final RecordFile file;
	private final Frame currentFrame;

	/**
	 * Reads a record of the state of the world from a file
	 * 
	 * @param path the source file
	 * @throws FileNotFoundException if the file couldn't be created
	 */
	public PhysicsPlayer(String path) throws FileNotFoundException {
		this.file = new RecordFile(path, true);
		currentFrame = new Frame(0);

		readDate();
	}

	private void readDate() {
		StringBuilder str = new StringBuilder();
		file.readLine(str);
		System.out.println("Reading physics record " + str);
	}

	public void newFrame(double timeStamp) {
		currentFrame.reset(timeStamp);
	}

	public Frame getCurrentFrame() {
		return currentFrame;
	}

	public void endOfFrame() {
		currentFrame.write(file);
	}

	/**
	 * Stops the recording and closes all in/out streams.
	 */
	public void close() {
		file.close();
	}

}
