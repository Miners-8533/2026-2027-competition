package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.EnumMap;
import java.util.Map;

/**
 * Part of the coordinator (specs/mechanisms.md section 11): the standing warnings and the one-off notices.
 * A warning is a standing condition, recomputed every update; each has a code and a sentence. The codes are what
 * other software acts on (the competition screen); the sentences are for the log and the verbose screen.
 */
public final class Warnings {
    public enum Code {
        COUNTER_NOT_READING, ENTRY_NOT_ANSWERING, FULL, OPPONENT_NECTAR, LAUNCH_MAP_EMPTY, FLYWHEEL_LOST, RAMP_NOT_CONFIGURED,
        BACKUP_MODE, STOW_WAITING, PUSHER_NOT_CALIBRATED, PUSHER_STALLED, PUSHER_JAMMED, ZERO_NOT_VERIFIED, ZERO_UNCERTAIN
    }

    private final Map<Code, String> now = new EnumMap<>(Code.class), next = new EnumMap<>(Code.class);
    private String notice;
    private double noticeUntil;

    /** Raise a warning for this update. */
    public void raise(Code code, String sentence) {
        next.put(code, sentence);
    }

    /** Ends the update: logs each warning when it appears and when it clears. */
    public void commit() {
        for (Map.Entry<Code, String> e : next.entrySet()) {
            if (!now.containsKey(e.getKey())) RunLog.event("WARNING " + e.getKey() + ": " + e.getValue());
        }
        for (Code c : now.keySet()) {
            if (!next.containsKey(c)) RunLog.event("WARNING cleared " + c);
        }
        now.clear();
        now.putAll(next);
        next.clear();
    }

    public boolean has(Code c) { return now.containsKey(c); }
    public int count() { return now.size(); }
    public Map<Code, String> all() { return now; }

    /** A one-off message, shown for a few seconds and logged. */
    public void notice(String text, double t) {
        notice = text;
        noticeUntil = t + MechParams.NOTICE_S;
        RunLog.event("NOTICE " + text);
    }

    /** The current notice, or null. */
    public String notice(double t) {
        return notice != null && t < noticeUntil ? notice : null;
    }
}
