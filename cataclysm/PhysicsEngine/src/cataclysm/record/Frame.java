package cataclysm.record;

import cataclysm.PhysicsWorld;
import cataclysm.broadphase.staticmeshes.StaticMesh;
import cataclysm.wrappers.RigidBody;

/**
 * Represents the recorded state of a group of object
 * 
 * @author Briac
 *
 */
public final class Frame implements ReadWriteObject {

	private float timeStamp;

	private ReadWriteList<RigidBodyRepr> addedBodies = new ReadWriteList<RigidBodyRepr>(RigidBodyRepr::new, null);
	private ReadWriteList<StaticMeshRepr> addedMeshes = new ReadWriteList<StaticMeshRepr>(StaticMeshRepr::new, null);

	private ReadWriteListPrimitive<Integer> removed = new ReadWriteListPrimitive<Integer>(f -> f.readInt(),
			(f, i) -> f.writeInt(i));

	private ReadWriteList<RigidBodyState> bodyStates = new ReadWriteList<RigidBodyState>(RigidBodyState::new, null);

	public Frame(PhysicsWorld world) {

	}

	public void fill(PhysicsWorld world) {
		addedBodies.rewind();
		for (RigidBody b : world.getBodies()) {
			b.fill(addedBodies.getNext());
		}

		addedMeshes.rewind();
		for(StaticMesh m : world.getMeshes()) {
			
		}
	}

	public Frame(RecordFile f) {
		read(f);
	}

	@Override
	public void read(RecordFile f) {
		timeStamp = f.readFloat();
		addedBodies.read(f);
		removed.read(f);
		bodyStates.read(f);
	}

	@Override
	public void write(RecordFile f) {
		f.writeFloat(timeStamp);
		addedBodies.write(f);
		removed.write(f);
		bodyStates.write(f);
	}

}
