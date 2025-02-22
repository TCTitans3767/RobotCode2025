package frc.robot;

public enum ReefBranch {
    A("A"),
    B("B"),
    C("C"),
    D("D"),
    E("E"),
    F("F"),
    G("G"),
    H("H"),
    I("I"),
    J("J"),
    K("K"),
    L("L");

    public static final ReefBranch fromString(String name) {
        for (ReefBranch branch : ReefBranch.values()) {
            if (branch.name == name) {
                return branch;
            }
        }
        return null;
    }

    private String name;

    private ReefBranch(String name) {
        this.name = name;
    }

    public String toString() {
        return this.name;
    }

    public boolean isLeft() {
        return (this == A || this == C || this == E || this == G || this == I || this == K);
    }

    public boolean isRight() {
        return (this == B || this == D || this == F || this == H || this == J || this == L);
    }
}
