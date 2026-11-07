package com.acmerobotics.dashboard.config;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/** PC stub of FTC Dashboard's @Config for tools/dash/DashSim: a marker that does nothing. */
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.TYPE)
public @interface Config {
    String value() default "";
}
