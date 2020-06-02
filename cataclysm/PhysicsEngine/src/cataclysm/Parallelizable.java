package cataclysm;

import static java.lang.annotation.ElementType.ANNOTATION_TYPE;
import static java.lang.annotation.ElementType.CONSTRUCTOR;
import static java.lang.annotation.ElementType.FIELD;
import static java.lang.annotation.ElementType.LOCAL_VARIABLE;
import static java.lang.annotation.ElementType.METHOD;
import static java.lang.annotation.ElementType.MODULE;
import static java.lang.annotation.ElementType.PACKAGE;
import static java.lang.annotation.ElementType.PARAMETER;
import static java.lang.annotation.ElementType.TYPE;
import static java.lang.annotation.ElementType.TYPE_PARAMETER;
import static java.lang.annotation.ElementType.TYPE_USE;
import static java.lang.annotation.RetentionPolicy.CLASS;

import java.lang.annotation.Documented;
import java.lang.annotation.Retention;
import java.lang.annotation.Target;

/**
 * This annotation indicates that a method can be called from multiple threads
 * at the same time when the physics engine <b>isn't</b> being updated.
 * Callbacks which are called when the physics engine is updated must also be
 * Parallelisable. <br>
 * Typical {@link Parallelizable} functions are
 * {@link PhysicsWorld#rayTest(RayTest)},
 * {@link PhysicsWorld#boxTriangleQuery(cataclysm.broadphase.AABB, java.util.Set)},
 * {@link PhysicsWorld#boxWrapperQuery(cataclysm.broadphase.AABB, java.util.Set)}
 * 
 * @author Briac
 *
 */
@Documented
@Retention(CLASS)
@Target({ TYPE, FIELD, METHOD, PARAMETER, CONSTRUCTOR, LOCAL_VARIABLE, ANNOTATION_TYPE, PACKAGE, TYPE_PARAMETER,
		TYPE_USE, MODULE })
public @interface Parallelizable {

}
