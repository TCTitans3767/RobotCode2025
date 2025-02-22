package frc.robot;

import java.util.EnumSet;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import frc.robot.utils.Logger;

public class ButtonBox {
    public class ButtonTopic {
        private String name;
        private BooleanTopic ntTopic;
        private BooleanEntry ntEntry;
        private AtomicBoolean atomicValue;

        protected ButtonTopic(String name) {
            this.name = name;
            this.ntTopic = table.getBooleanTopic(name);
            this.ntEntry = ntTopic.getEntry(false);
            this.atomicValue = new AtomicBoolean();

            Consumer<NetworkTableEvent> onEvent = event -> { remoteSetActive(event.valueData.value.getBoolean()); };

            ntInstance.addListener(ntEntry, EnumSet.of(NetworkTableEvent.Kind.kValueRemote), onEvent);
            this.ntTopic.publish();
        }

        public String getName() {
            return name;
        }

        public boolean isActive() {
            return this.atomicValue.get();
        }

        protected void remoteSetActive(boolean active) {
            if (isActive() != active) {
                this.atomicValue.set(active);
            }
        }

        protected void setActive(boolean active) {
            if (isActive() != active) {
                this.atomicValue.set(active);
                this.ntEntry.set(active);
            }
        }
    }

    public class BranchButtonTopic extends ButtonTopic {
        protected BranchButtonTopic(String name) {
            super(name);
        }

        @Override
        protected void remoteSetActive(boolean active) {
            super.remoteSetActive(active);
            setSelectedBranch(ReefBranch.fromString(getName()));
        }

        @Override
        protected void setActive(boolean active) {
            super.setActive(active);
            setSelectedBranch(ReefBranch.fromString(getName()));
        }
    }

    public class LevelButtonTopic extends ButtonTopic {
        protected LevelButtonTopic(String name) {
            super(name);
        }

        @Override
        protected void remoteSetActive(boolean active) {
            super.remoteSetActive(active);
            setSelectedLevel(ReefLevel.fromString(getName()));
        }

        @Override
        protected void setActive(boolean active) {
            super.setActive(active);
            setSelectedLevel(ReefLevel.fromString(getName()));
        }
    }

    public class SelectedReefBranchTopic {
        private StringTopic ntTopic;
        private StringEntry ntEntry;
        private AtomicReference<ReefBranch> atomicValue;

        protected SelectedReefBranchTopic() {
            this.ntTopic = table.getStringTopic(Constants.NetworkTables.selectedReefBranchTopic);
            this.ntEntry = ntTopic.getEntry(Constants.NetworkTables.initialReefBranch.toString());
            this.atomicValue = new AtomicReference<ReefBranch>();

            Consumer<NetworkTableEvent> onEvent = event -> {
                String branchName = event.valueData.value.getString();
                ReefBranch branch = ReefBranch.fromString(branchName);
                if (branch == null) {
                    Logger.logSystemError("ButtonBox: Received invalid branch name via networktables: " + branchName);
                    return;
                }
                remoteSetBranch(branch);
            };

            ntInstance.addListener(ntEntry, EnumSet.of(NetworkTableEvent.Kind.kValueRemote), onEvent);
            this.ntTopic.publish();
        }

        protected ReefBranch getBranch() {
            return this.atomicValue.get();
        }

        protected void remoteSetBranch(ReefBranch branch) {
            if (getBranch() != branch) {
                this.atomicValue.set(branch);
                setSelectedBranch(branch);
            }
        }

        protected void setBranch(ReefBranch branch) {
            if (getBranch() != branch) {
                this.atomicValue.set(branch);
                ntEntry.set(branch.toString());
                setSelectedBranch(branch);
            }
        }
    }

    public class SelectedReefLevelTopic {
        private StringTopic ntTopic;
        private StringEntry ntEntry;
        private AtomicReference<ReefLevel> atomicValue;

        protected SelectedReefLevelTopic() {
            this.ntTopic = table.getStringTopic(Constants.NetworkTables.selectedReefBranchTopic);
            this.ntEntry = ntTopic.getEntry(Constants.NetworkTables.initialReefBranch.toString());
            this.atomicValue = new AtomicReference<ReefLevel>();

            Consumer<NetworkTableEvent> onEvent = event -> {
                String levelName = event.valueData.value.getString();
                ReefLevel level = ReefLevel.fromString(levelName);
                if (level == null) {
                    Logger.logSystemError("ButtonBox: Received invalid level name via networktables: " + levelName);
                    return;
                }
                remoteSetLevel(level);
            };

            ntInstance.addListener(ntEntry, EnumSet.of(NetworkTableEvent.Kind.kValueRemote), onEvent);
            this.ntTopic.publish();
        }

        protected ReefLevel getLevel() {
            return this.atomicValue.get();
        }

        protected void remoteSetLevel(ReefLevel level) {
            if (getLevel() != level) {
                this.atomicValue.set(level);
                setSelectedLevel(level);
            }
        }

        protected void setLevel(ReefLevel level) {
            if (getLevel() != level) {
                this.atomicValue.set(level);
                ntEntry.set(level.toString());
                setSelectedLevel(level);
            }
        }
    }

    private static ButtonBox instance;

    private EventLoop eventLoop;
    private NetworkTableInstance ntInstance;
    private NetworkTable table;
    private Map<ReefBranch, ButtonTopic> branchButtonTopics;
    private Map<ReefLevel, ButtonTopic> levelButtonTopics;
    private ButtonTopic ejectAlgaeButtonTopic;
    private ButtonTopic coralStationAlignRightTopic;
    private SelectedReefBranchTopic selectedReefBranchTopic;
    private SelectedReefLevelTopic selectedReefLevelTopic;

    public ButtonBox(GenericHID controller) {
        instance = this;
        eventLoop = new EventLoop();
        ntInstance = NetworkTableInstance.getDefault();
        table = ntInstance.getTable(Constants.NetworkTables.buttonBoxTable);
        branchButtonTopics = new HashMap<ReefBranch, ButtonTopic>();
        levelButtonTopics = new HashMap<ReefLevel, ButtonTopic>();
        ejectAlgaeButtonTopic = new ButtonTopic(Constants.NetworkTables.ejectAlgaeTopic);
        coralStationAlignRightTopic = new ButtonTopic(Constants.NetworkTables.coralStationAlignRightTopic);
        selectedReefBranchTopic = new SelectedReefBranchTopic();
        selectedReefLevelTopic = new SelectedReefLevelTopic();

        for (ReefBranch branch : ReefBranch.values()) {
            ButtonTopic topic = new BranchButtonTopic(branch.toString());
            branchButtonTopics.put(branch, topic);

            int buttonNumber = Constants.ButtonBoxButtons.branchButtonMap.get(branch);
            BooleanEvent event = controller.button(buttonNumber, eventLoop).debounce(Constants.ButtonBoxButtons.debounceSeconds);
            event.ifHigh(() -> { topic.setActive(true); });
        }
        for (ReefLevel level : ReefLevel.values()) {
            ButtonTopic topic = new LevelButtonTopic(level.toString());
            levelButtonTopics.put(level, topic);

            int buttonNumber = Constants.ButtonBoxButtons.levelButtonMap.get(level);
            BooleanEvent event = controller.button(buttonNumber, eventLoop).debounce(Constants.ButtonBoxButtons.debounceSeconds);
            event.ifHigh(() -> { topic.setActive(true); });
        }
    }

    public void periodic() {
        eventLoop.poll();
    }

    public static boolean isLeftBranchSelected() {
        return getSelectedBranch().isLeft();
    }

    public static boolean isRightBranchSelected() {
        return getSelectedBranch().isRight();
    }

    public static boolean isEjectAlgaeEnabled() {
        return instance.ejectAlgaeButtonTopic.isActive();
    }

    public static boolean isCoralStationAlignRight() {
        return instance.coralStationAlignRightTopic.isActive();
    }

    public static ReefBranch getSelectedBranch() {
        return instance.selectedReefBranchTopic.getBranch();
    }

    public static void setSelectedBranch(ReefBranch branch) {
        ReefBranch lastBranch = getSelectedBranch();

        instance.branchButtonTopics.get(lastBranch).setActive(false);
        instance.branchButtonTopics.get(branch).setActive(true);
        instance.selectedReefBranchTopic.setBranch(branch);
    }

    public static ReefLevel getSelectedLevel() {
        return instance.selectedReefLevelTopic.getLevel();
    }

    public static void setSelectedLevel(ReefLevel level) {
        instance.selectedReefLevelTopic.setLevel(level);
        ReefLevel lastLevel = getSelectedLevel();

        instance.levelButtonTopics.get(lastLevel).setActive(false);
        instance.levelButtonTopics.get(level).setActive(true);
        instance.selectedReefLevelTopic.setLevel(level);
    }
}
