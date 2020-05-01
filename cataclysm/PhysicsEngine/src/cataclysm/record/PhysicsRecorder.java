package cataclysm.record;

import java.io.FileNotFoundException;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Date;

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

	/**
	 * Records the state of the world in a file
	 * 
	 * @param path the destination file
	 * @throws FileNotFoundException if the file couldn't be created
	 */
	public PhysicsRecorder(String path) throws FileNotFoundException {
		this.file = new RecordFile(path, false);
		currentFrame = new Frame(0);

		writeDate();
	}

	private void writeDate() {
		DateFormat df = new SimpleDateFormat("MM/dd/yyyy HH:mm:ss");
		Date today = Calendar.getInstance().getTime();
		String reportDate = df.format(today);
		file.writeLine(reportDate);
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
		file.flushAndClose();
	}

}
