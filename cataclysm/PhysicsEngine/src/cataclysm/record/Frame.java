package cataclysm.record;

import java.util.Map;

import cataclysm.PhysicsWorld;
import cataclysm.broadphase.staticmeshes.StaticMesh;
import cataclysm.broadphase.staticmeshes.StaticMeshManager;
import cataclysm.wrappers.RigidBody;
import cataclysm.wrappers.RigidBodyManager;

/**
 * Represents the recorded state of a group of objects
 * 
 * @author Briac
 *
 */
public final class Frame implements ReadWriteObject {
	
	/**
	 * The total number of bytes of this frame, including the header
	 */
	private int frameSize = 0;
	private boolean filled = false;

	private final ReadWriteList<RigidBodyRepr> addedBodies = new ReadWriteList<RigidBodyRepr>(RigidBodyRepr::new,
			RigidBodyRepr::new);
	private final ReadWriteList<StaticMeshRepr> addedMeshes = new ReadWriteList<StaticMeshRepr>(StaticMeshRepr::new,
			StaticMeshRepr::new);

	private final ReadWriteList<RigidBodyRepr> removedBodies = new ReadWriteList<RigidBodyRepr>(RigidBodyRepr::new,
			RigidBodyRepr::new);
	private final ReadWriteList<StaticMeshRepr> removedMeshes = new ReadWriteList<StaticMeshRepr>(StaticMeshRepr::new,
			StaticMeshRepr::new);

	private final ReadWriteList<RigidBodyState> bodyStates = new ReadWriteList<RigidBodyState>(RigidBodyState::new,
			RigidBodyState::new);

	public Frame() {

	}
	
	public void updateAddedAndRemoved(StaticMeshManager meshes, RigidBodyManager bodies, boolean reversed, Map<Long, StaticMesh> addedMeshes, Map<Long, RigidBody> addedBodies) {
		if(!reversed) {
			for(StaticMeshRepr repr : this.removedMeshes) {
				meshes.removeElement(addedMeshes.remove(repr.ID).getID());
			}
			for(StaticMeshRepr repr : this.addedMeshes) {
				StaticMesh m = meshes.newMesh(repr);
				addedMeshes.put(repr.ID, m);
			}
			for(RigidBodyRepr repr : this.removedBodies) {
				bodies.removeElement(addedBodies.remove(repr.ID).getID());
			}
			for(RigidBodyRepr repr : this.addedBodies) {
				RigidBody b = bodies.newBody(repr);
				addedBodies.put(repr.ID, b);
				
				//Disable all external forces
				b.setInvMass(0);
				b.setExternalForces(false);
				b.setSkipIntegration(true);
				b.setRotationBlocked(true);
			}
		}else {
			for(StaticMeshRepr repr : this.addedMeshes) {
				meshes.removeElement(addedMeshes.remove(repr.ID).getID());
			}
			for(StaticMeshRepr repr : this.removedMeshes) {
				StaticMesh m = meshes.newMesh(repr);
				addedMeshes.put(repr.ID, m);
			}
			for(RigidBodyRepr repr : this.addedBodies) {
				bodies.removeElement(addedBodies.remove(repr.ID).getID());
			}
			for(RigidBodyRepr repr : this.removedBodies) {
				RigidBody b = bodies.newBody(repr);
				addedBodies.put(repr.ID, b);
				
				//Disable all external forces
				b.setInvMass(0);
				b.setExternalForces(false);
				b.setSkipIntegration(true);
				b.setRotationBlocked(true);
			}
		}
	}

	public void reset() {
		frameSize = 0;
		addedMeshes.rewind();
		removedMeshes.rewind();
		addedBodies.rewind();
		removedBodies.rewind();
		bodyStates.rewind();
		filled = false;
	}

	public boolean isFilled() {
		return filled;
	}

	public void fillMeshes(Iterable<StaticMesh> added, Iterable<StaticMesh> removed) {
		for (StaticMesh m : added) {
			m.fill(addedMeshes.getNext());
		}
		for (StaticMesh m : removed) {
			m.fill(removedMeshes.getNext());
		}
	}

	public void fillBodies(Iterable<RigidBody> added, Iterable<RigidBody> removed) {
		for (RigidBody b : added) {
			b.fill(addedBodies.getNext());
		}
		for (RigidBody b : removed) {
			b.fill(removedBodies.getNext());
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

	private int readHeader(RecordFile f) {
		frameSize = f.readInt();
		return frameSize;
	}

	@Override
	public void read(RecordFile f) {
		int p = f.getPosition();
		
		int startFrameSize = readHeader(f);
		System.out.println("Reading frame: size = " + frameSize + " bytes");
		addedMeshes.read(f);
		removedMeshes.read(f);
		addedBodies.read(f);
		removedBodies.read(f);
		bodyStates.read(f);
		int endFrameSize = readHeader(f);
		
		int bytesRead = f.getPosition() - p;
		if(startFrameSize != endFrameSize) {
			throw new IllegalStateException("Error when reading frame, size headers don't match at " + p);
		}
		if(bytesRead != frameSize) {
			throw new IllegalStateException("Error when reading frame, read " + bytesRead + " bytes instead of " + frameSize);
		}

		filled = true;
	}

	@Override
	public void write(RecordFile f) {
		frameSize = size();
		System.out.println("Writing frame: size = " + frameSize + " bytes");
		int startPosition = f.getPosition();
		f.writeInt(frameSize);// write the size in the begining
		addedMeshes.write(f);
		removedMeshes.write(f);
		addedBodies.write(f);
		removedBodies.write(f);
		bodyStates.write(f);
		f.writeInt(frameSize);// write the size at the end as well
		int endPosition = f.getPosition();
		
		int bytesWritten = endPosition - startPosition;
		
		if(bytesWritten != frameSize) {
			throw new IllegalStateException("Wrote " + bytesWritten + " instead of " + frameSize);
		}
	}

	@Override
	public int size() {
		int frameSize = addedMeshes.size() + removedMeshes.size() + addedBodies.size() + removedBodies.size()
				+ bodyStates.size();
		return frameSize + headerSize() * 2;// one mark in the begining and the other at the end
	}

	public int headerSize() {
		return 4;// size of an int
	}
	
	ReadWriteList<RigidBodyState> getBodyStates(){
		return bodyStates;
	}

}
