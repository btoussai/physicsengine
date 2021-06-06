package cataclysm.annotations;

import static java.lang.annotation.ElementType.FIELD;
import static java.lang.annotation.ElementType.METHOD;
import static java.lang.annotation.RetentionPolicy.CLASS;

import java.lang.annotation.Documented;
import java.lang.annotation.Retention;
import java.lang.annotation.Target;

/**
 * This annotation indicates that a public field or that the return value of a method is safe
 * to read but must not be modified.
 * 
 * @author Briac Toussaint
 *
 */
@Documented
@Retention(CLASS)
@Target({ FIELD, METHOD })
public @interface ReadOnly {

}
