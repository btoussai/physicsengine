package cataclysm.wrappers;

import cataclysm.record.ReadWriteObject;
import cataclysm.record.RecordFile;

/**
 * Contient les informations permettant de calculer la masse d'un wrapper.
 * 
 * @author Briac
 *
 */
public final class MassProperties implements ReadWriteObject {

	private float mass;
	private float volume;
	private float surfaceArea;
	private boolean hollow;
	private float density;
	private boolean useDensity = true;

	public MassProperties(float volume, float surfaceArea, boolean hollow, float density) {
		this.volume = volume;
		this.surfaceArea = surfaceArea;
		this.hollow = hollow;
		this.density = density;
	}

	public MassProperties(float mass, float volume, float surfaceArea, boolean hollow) {
		this.mass = mass;
		this.volume = volume;
		this.surfaceArea = surfaceArea;
		this.hollow = hollow;
	}

	public MassProperties(MassProperties other) {
		set(other);
	}

	public void scale(float scaleFactor) {
		mass *= scaleFactor;
		surfaceArea *= scaleFactor * scaleFactor;
		volume *= scaleFactor * scaleFactor * scaleFactor;
	}

	public float computeMass() {
		if (useDensity) {
			mass = density * (hollow ? surfaceArea : volume);
		} else {
			density = mass / (hollow ? surfaceArea : volume);
		}
		return mass;
	}

	public float getMass() {
		return mass;
	}

	public void setMass(float mass) {
		this.mass = mass;
	}

	public float getVolume() {
		return volume;
	}

	public void setVolume(float volume) {
		this.volume = volume;
	}

	public float getSurfaceArea() {
		return surfaceArea;
	}

	public void setSurfaceArea(float surfaceArea) {
		this.surfaceArea = surfaceArea;
	}

	public boolean isHollow() {
		return hollow;
	}

	public void setHollow(boolean hollow) {
		this.hollow = hollow;
	}

	public float getDensity() {
		return density;
	}

	public void setDensity(float density) {
		this.density = density;
	}

	/**
	 * Indique si la masse est calcul�e � partir de la densit� ou l'inverse.
	 * 
	 * @return true si la masse est calcul�e � partir de la densit�, false si la
	 *         densit� est calcul�e � partir de la masse.
	 */
	public boolean useDensity() {
		return useDensity;
	}

	/**
	 * Change la fa�on dont sont calcul�es la masse et la densit�.
	 * 
	 * @param useDensity <br>
	 *                   Si true --> la masse est calcul�e � partir de la
	 *                   densit�.<br>
	 *                   Si false --> la densit� est calcul�e � partir de la masse.
	 */
	public void setUseDensity(boolean useDensity) {
		this.useDensity = useDensity;
	}

	@Override
	public void read(RecordFile f) {
		mass = f.readFloat();
		volume = f.readFloat();
		surfaceArea = f.readFloat();
		hollow = f.readBool();
		density = f.readFloat();
		useDensity = f.readBool();
	}

	@Override
	public void write(RecordFile f) {
		f.writeFloat(mass);
		f.writeFloat(volume);
		f.writeFloat(surfaceArea);
		f.writeBool(hollow);
		f.writeFloat(density);
		f.writeBool(useDensity);
	}

	public void set(MassProperties other) {
		this.mass = other.mass;
		this.volume = other.volume;
		this.surfaceArea = other.surfaceArea;
		this.hollow = other.hollow;
		this.density = other.density;
		this.useDensity = other.useDensity;
	}

	@Override
	public int size() {
		return 4 + 4 + 4 + 1 + 4 + 1;
	}

}
