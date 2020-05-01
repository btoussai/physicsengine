package cataclysm.record;

import java.util.List;

import cataclysm.PhysicsWorld;
import cataclysm.broadphase.staticmeshes.StaticMesh;
import cataclysm.wrappers.RigidBody;

/**
 * Represents the recorded state of a group of objects
 * 
 * @author Briac
 *
 */
public final class Frame implements ReadWriteObject {

	private double timeStamp;

	private final ReadWriteList<RigidBodyRepr> addedBodies = new ReadWriteList<RigidBodyRepr>(RigidBodyRepr::new,
			RigidBodyRepr::new);
	private final ReadWriteList<StaticMeshRepr> addedMeshes = new ReadWriteList<StaticMeshRepr>(StaticMeshRepr::new,
			StaticMeshRepr::new);

	private final ReadWriteListPrimitive<Long> removedBodies = new ReadWriteListPrimitive<Long>(f -> f.readLong(),
			(f, i) -> f.writeLong(i), 8);
	private final ReadWriteListPrimitive<Long> removedMeshes = new ReadWriteListPrimitive<Long>(f -> f.readLong(),
			(f, i) -> f.writeLong(i), 8);

	private final ReadWriteList<RigidBodyState> bodyStates = new ReadWriteList<RigidBodyState>(RigidBodyState::new,
			RigidBodyState::new);

	public Frame(double d) {
		this.timeStamp = d;
	}

	public void reset(double timeStamp) {
		this.timeStamp = timeStamp;
		addedMeshes.rewind();
		removedMeshes.clear();
		addedBodies.rewind();
		removedBodies.clear();
		bodyStates.rewind();
	}

	public void fillMeshes(List<StaticMesh> added, List<StaticMesh> removed) {
		for (StaticMesh m : added) {
			m.fill(addedMeshes.getNext());
		}
		for (StaticMesh m : removed) {
			removedMeshes.add(m.getID());
		}
	}

	public void fillBodies(List<RigidBody> added, List<RigidBody> removed) {
		for (RigidBody b : added) {
			b.fill(addedBodies.getNext());
		}
		for (RigidBody b : removed) {
			removedBodies.add(b.getID());
		}
	}

	public void fillBodiesStates(PhysicsWorld world) {
		bodyStates.rewind();
		for (RigidBody b : world.getBodies()) {
			b.fill(bodyStates.getNext());
		}
	}

	public Frame(RecordFile f) {
		read(f);
	}

	@Override
	public void read(RecordFile f) {
		timeStamp = f.readDouble();
		addedMeshes.read(f);
		removedMeshes.read(f);
		addedBodies.read(f);
		removedBodies.read(f);
		bodyStates.read(f);
	}

	@Override
	public void write(RecordFile f) {
		System.out.println("Writing frame: time = " + timeStamp + " size = " + size() + " bytes");

		f.writeDouble(timeStamp);
		addedMeshes.write(f);
		removedMeshes.write(f);
		addedBodies.write(f);
		removedBodies.write(f);
		bodyStates.write(f);
	}

	@Override
	public int size() {
		return 8 + addedMeshes.size() + removedMeshes.size() + addedBodies.size() + removedBodies.size()
				+ bodyStates.size();
	}

}
