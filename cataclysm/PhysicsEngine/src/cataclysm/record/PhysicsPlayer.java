package cataclysm.record;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

import cataclysm.broadphase.staticmeshes.StaticMesh;
import cataclysm.broadphase.staticmeshes.StaticMeshManager;
import cataclysm.wrappers.RigidBody;
import cataclysm.wrappers.RigidBodyManager;
import math.Clamp;
import math.MatrixOps;
import math.vector.Matrix3f;
import math.vector.Vector3f;

/**
 * This class enables to play back a record of a simulation.
 * 
 * @see PhysicsRecorder
 * 
 * @author Briac
 *
 */
public class PhysicsPlayer {

	public enum PlaybackMode {
		CLAMP, BOUNCE, LOOP;
	}

	private final RecordFile file;
	private int totalFrameCount = 0;
	private int currentFrameIndex = 0;
	private int nextFrameIndex = 0;
	private final PlaybackMode mode;
	private Frame currentFrame = new Frame();
	private Frame nextFrame = new Frame();

	/**
	 * This map establishes a mapping between the objects in the record and the
	 * objects in the simulation
	 */
	private final Map<Long, StaticMesh> addedMeshes = new HashMap<Long, StaticMesh>();
	/**
	 * This map establishes a mapping between the objects in the record and the
	 * objects in the simulation
	 */
	private final Map<Long, RigidBody> addedBodies = new HashMap<Long, RigidBody>();

	/**
	 * The time that has passed in the record, expressed in frame count.
	 */
	private double currentTime = 0;

	/**
	 * Defines the relation between a time step as perceived in the simulation and a
	 * time step in the record.<br>
	 * {@code recordTimeStep = playbackSpeed * simulationTimeStep}
	 */
	private float playbackSpeed = 1;

	/**
	 * This boolean becomes true whenever a new frame is loaded
	 */
	private boolean crossedFrame = false;

	private boolean initDone = false;

	/**
	 * Reads a record of the state of the world from a file
	 * 
	 * @param path          the source file
	 * @param mode          how the player behaves when an end of the record is
	 *                      reached.
	 * @param playbackSpeed The rate at which the record is played (in the range [0,
	 *                      1])
	 * 
	 * @see PhysicsPlayer#getPlaybackSpeed()
	 * @see PhysicsPlayer#setPlaybackSpeed(float)
	 * @throws IOException
	 */
	public PhysicsPlayer(String path, PlaybackMode mode, float playbackSpeed) throws IOException {
		this.file = new RecordFile(path, true);
		this.mode = mode;
		setPlaybackSpeed(playbackSpeed);
	}

	/**
	 * Initializes the record from its start or its end depending on the sign of the
	 * playback speed. <br>
	 * Note that +0.0f and -0.0f are treated differently.
	 * 
	 * @see PhysicsPlayer#getPlaybackSpeed()
	 * @see PhysicsPlayer#setPlaybackSpeed(float)
	 * 
	 */
	private void init() {
		file.seek(0);
		String date = readDate();
		readFrameCount();

		if (!initDone) {
			System.out.println("Date: " + date);
			System.out.println("Total frame count: " + totalFrameCount);
		}

		if (!reversed()) {
			currentFrame.read(file);
			nextFrame.read(file);

			currentTime = 0;
			currentFrameIndex = 0;
			nextFrameIndex = 1;
		} else {
			for (int i = 0; i < totalFrameCount - 2; i++) {
				int nextFrameSize = file.peekInt();
				file.skipBytes(nextFrameSize);
			}
			nextFrame.read(file);
			currentFrame.read(file);
			file.skipBytes(-currentFrame.size());

			nextFrameIndex = totalFrameCount - 2;
			currentFrameIndex = totalFrameCount - 1;
			currentTime = currentFrameIndex;
		}

		initDone = true;
		crossedFrame = true;
	}

	/**
	 * Steps the playback. No more than one frame can be stepped, either forward or
	 * backward.
	 * 
	 * @param bodies
	 * @param meshes
	 * 
	 */
	public void step(StaticMeshManager meshes, RigidBodyManager bodies) {
		if (!initDone) {
			init();
		} else {
			currentTime += playbackSpeed;
		}

		refillFrames();
		interpolateFrames(meshes, bodies);

		crossedFrame = false;
	}

	private void interpolateFrames(StaticMeshManager meshes, RigidBodyManager bodies) {

		if (crossedFrame) {
			currentFrame.updateAddedAndRemoved(meshes, bodies, reversed(), addedMeshes, addedBodies);
		}

		float blend = (float) (currentTime % 1.0);
		if (reversed()) {
			blend = 1.0f - blend;
		}

		System.out.println("Blend: " + blend);

		if (blend < 0.0) {
			for (RigidBodyState state : currentFrame.getBodyStates()) {
				RigidBody b = addedBodies.get(state.ID);
				b.getOriginTransform().getRotation().load(state.rotation);
				b.getOriginTransform().getTranslation().set(state.position);
				b.getVelocity().set(state.velocity).scale(playbackSpeed);
				b.getAngularVelocity().set(state.angularVelocity).scale(playbackSpeed);

			}
		} else {
			float dt = 1.0f / 60.0f;
			Vector3f axis = new Vector3f();
			Matrix3f deltaRotation = new Matrix3f();
			for (RigidBodyState state : currentFrame.getBodyStates()) {
				RigidBody b = addedBodies.get(state.ID);
				b.getVelocity().set(state.velocity).scale(playbackSpeed);
				b.getAngularVelocity().set(state.angularVelocity).scale(playbackSpeed);

				b.getOriginTransform().getTranslation().set(state.position).translate(b.getVelocity(), blend * dt);

				axis.set(b.getAngularVelocity());
				float omega2 = axis.lengthSquared();
				if (omega2 > 1E-6f) {

					float omega = (float) Math.sqrt(omega2);
					float one_over_omega = 1.0f / omega;
					axis.set(axis.x * one_over_omega, axis.y * one_over_omega, axis.z * one_over_omega);
					MatrixOps.createRotationMatrix3f(omega * blend * dt, axis, deltaRotation);
					Matrix3f.mul(deltaRotation, state.rotation, b.getOriginTransform().getRotation());
				} else {
					b.getOriginTransform().getRotation().load(state.rotation);
				}
			}
		}

	}

	/**
	 * Modifies currentTime according to the playback speed and then <br>
	 * checks currentTime against the record boundaries. <br>
	 * Loads a new frame if necessary
	 */
	private void refillFrames() {
		System.out.println("Current time: " + currentTime);

		if (mode == PlaybackMode.CLAMP) {
			currentTime = Clamp.clamp(currentTime, 0.0, totalFrameCount - 1);
		} else if (mode == PlaybackMode.BOUNCE) {
			if (currentTime < 0) {
				currentTime = -currentTime;
				playbackSpeed = Math.abs(playbackSpeed);
				swapFrames();
				crossedFrame = true;
			} else if (currentTime > totalFrameCount - 1) {
				currentTime = (totalFrameCount - 1) + (totalFrameCount - 1) - currentTime;
				playbackSpeed = -Math.abs(playbackSpeed);
				swapFrames();
				crossedFrame = true;
			}
		} else if (mode == PlaybackMode.LOOP) {
			if (currentTime < 0) {
				currentTime += totalFrameCount - 1;
				init();
			} else if (currentTime > totalFrameCount - 1) {
				currentTime -= totalFrameCount - 1;
				init();
			}
		}

		if (playbackSpeed > 0) {
			if (this.currentTime >= nextFrameIndex) {
				currentFrameIndex = nextFrameIndex;
				nextFrameIndex++;

				System.out.println("Reading frame " + nextFrameIndex);
				swapFrames();
				crossedFrame = true;
				if (nextFrameIndex != totalFrameCount) {
					nextFrame.read(file);
				}

			}
		} else if (playbackSpeed < 0) {
			if (this.currentTime <= nextFrameIndex) {
				currentFrameIndex = nextFrameIndex;
				nextFrameIndex--;

				if (nextFrameIndex != -1) {
					System.out.println("Reading frame " + nextFrameIndex);
					file.skipBytes(-nextFrame.size());// rollback till the start of next frame
					file.skipBytes(-4);
					int frameSize = file.readInt();// read the size of the previous frame in the file
					file.skipBytes(-frameSize);// rollback one frame in the file
					swapFrames();
					nextFrame.read(file);
				}
				crossedFrame = true;
			}
		}

	}

	private void swapFrames() {
		Frame temp = currentFrame;
		currentFrame = nextFrame;
		nextFrame = temp;
	}

	private String readDate() {
		StringBuilder str = new StringBuilder();
		file.readLine(str);
		return str.toString();
	}

	private void readFrameCount() {
		totalFrameCount = file.readInt();
	}

	/**
	 * Stops the recording and closes all in/out streams.
	 */
	public void close() {
		file.close();
		System.out.println("Closing RecordFile");
	}

	/**
	 * @return the rate at which the time passes in the record, between -1 and 1
	 */
	public float getPlaybackSpeed() {
		return playbackSpeed;
	}

	/**
	 * @param playbackSpeed the rate at which the time passes in the record, between
	 *                      -1 and 1
	 */
	public void setPlaybackSpeed(float playbackSpeed) {
		if (playbackSpeed < -1 || playbackSpeed > 1) {
			throw new IllegalArgumentException("The playback speed must be in [-1, 1]");
		}
		boolean wasReversed = reversed();
		this.playbackSpeed = playbackSpeed;

		if (wasReversed != reversed()) {// reading direction has changed
			swapFrames();
		}
	}

	public int getCurrentFrameIndex() {
		return currentFrameIndex;
	}

	public int getNextFrameIndex() {
		return nextFrameIndex;
	}

	private boolean reversed() {
		return Double.compare(playbackSpeed, +0.0) < 0;
	}
}
