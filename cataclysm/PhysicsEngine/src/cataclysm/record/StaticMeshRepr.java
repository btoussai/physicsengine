package cataclysm.record;

import cataclysm.contact_creation.ContactProperties;
import math.vector.Vector3f;

public final class StaticMeshRepr implements ReadWriteObject {

	public long ID;
	public final ReadWriteList<TriangleRepr> triangles = new ReadWriteList<TriangleRepr>(TriangleRepr::new, TriangleRepr::new);
	public final Vector3f min = new Vector3f();
	public final Vector3f max = new Vector3f();
	public final ContactProperties contactProperties = new ContactProperties(0, 0);


	public StaticMeshRepr() {
	}
	
	public StaticMeshRepr(RecordFile f) {
		read(f);
	}
	
	@Override
	public void read(RecordFile f) {
		ID = f.readLong();
		triangles.read(f);
		f.readVector3f(min);
		f.readVector3f(max);
		contactProperties.read(f);
	}

	@Override
	public void write(RecordFile f) {
		f.writeLong(ID);
		triangles.write(f);
		f.writeVector3f(min);
		f.writeVector3f(max);
		contactProperties.write(f);
	}

	@Override
	public int size() {
		return 8 + triangles.size() + 2*3*4 + contactProperties.size();
	}

}
