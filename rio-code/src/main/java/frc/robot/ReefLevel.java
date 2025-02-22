package frc.robot;

public enum ReefLevel {
    L1("1"),
    L2("2"),
    L3("3"),
    L4("4");

    public static final ReefLevel fromString(String name) {
        for (ReefLevel level : ReefLevel.values()) {
            if (level.name == name) {
                return level;
            }
        }
        return null;
    }

    private String name;

    private ReefLevel(String name) {
        this.name = name;
    }

    public String toString() {
        return this.name;
    }
    
}
