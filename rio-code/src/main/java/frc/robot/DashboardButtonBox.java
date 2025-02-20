package frc.robot;

import java.util.EnumSet;
import java.util.concurrent.atomic.AtomicBoolean;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.PubSubOptions;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.Utils.ReefPosition;

public class DashboardButtonBox {

    private static ReefPosition selectedReef = ReefPosition.A;
    private static int selectedLevel = 0;

    private static BooleanEntry A;
    private static BooleanEntry B;
    private static BooleanEntry C;
    private static BooleanEntry D;
    private static BooleanEntry E;
    private static BooleanEntry F;
    private static BooleanEntry G;
    private static BooleanEntry H;
    private static BooleanEntry I;
    private static BooleanEntry J;
    private static BooleanEntry K;
    private static BooleanEntry L;

    private static BooleanEntry L1;
    private static BooleanEntry L2;
    private static BooleanEntry L3;
    private static BooleanEntry L4;

    private static BooleanEntry knockOffAlgae;

    private static AtomicBoolean AValue = new AtomicBoolean();
    private static AtomicBoolean BValue = new AtomicBoolean();
    private static AtomicBoolean CValue = new AtomicBoolean();
    private static AtomicBoolean DValue = new AtomicBoolean();
    private static AtomicBoolean EValue = new AtomicBoolean();
    private static AtomicBoolean FValue = new AtomicBoolean();
    private static AtomicBoolean GValue = new AtomicBoolean();
    private static AtomicBoolean HValue = new AtomicBoolean();
    private static AtomicBoolean IValue = new AtomicBoolean();
    private static AtomicBoolean JValue = new AtomicBoolean();
    private static AtomicBoolean KValue = new AtomicBoolean();
    private static AtomicBoolean LValue = new AtomicBoolean();

    private static AtomicBoolean L1Value = new AtomicBoolean();
    private static AtomicBoolean L2Value = new AtomicBoolean();
    private static AtomicBoolean L3Value = new AtomicBoolean();
    private static AtomicBoolean L4Value = new AtomicBoolean();

    private static AtomicBoolean knockOffAlgaeValue = new AtomicBoolean();

    private static NetworkTableInstance instance;
    private static NetworkTable table;
    
    public DashboardButtonBox() {
        instance = NetworkTableInstance.getDefault();
        table = instance.getTable("Dashboard ButtonBox");

        A = table.getBooleanTopic("A").getEntry(false);
        B = table.getBooleanTopic("B").getEntry(false);
        C = table.getBooleanTopic("C").getEntry(false);
        D = table.getBooleanTopic("D").getEntry(false);
        E = table.getBooleanTopic("E").getEntry(false);
        F = table.getBooleanTopic("F").getEntry(false);
        G = table.getBooleanTopic("G").getEntry(false);
        H = table.getBooleanTopic("H").getEntry(false);
        I = table.getBooleanTopic("I").getEntry(false);
        J = table.getBooleanTopic("J").getEntry(false);
        K = table.getBooleanTopic("K").getEntry(false);
        L = table.getBooleanTopic("L").getEntry(false);
        L1 = table.getBooleanTopic("L1").getEntry(false);
        L2 = table.getBooleanTopic("L2").getEntry(false);
        L3 = table.getBooleanTopic("L3").getEntry(false);
        L4 = table.getBooleanTopic("L4").getEntry(false);

        knockOffAlgae = table.getBooleanTopic("Knock Off Algae").getEntry(false);

        instance.addListener(A, EnumSet.of(NetworkTableEvent.Kind.kValueRemote), event -> {
            AValue.set(event.valueData.value.getBoolean());
        });
        instance.addListener(B, EnumSet.of(NetworkTableEvent.Kind.kValueRemote), event -> {
            BValue.set(event.valueData.value.getBoolean());
        });
        instance.addListener(C, EnumSet.of(NetworkTableEvent.Kind.kValueRemote), event -> {
            CValue.set(event.valueData.value.getBoolean());
        });
        instance.addListener(D, EnumSet.of(NetworkTableEvent.Kind.kValueRemote), event -> {
            DValue.set(event.valueData.value.getBoolean());
        });
        instance.addListener(E, EnumSet.of(NetworkTableEvent.Kind.kValueRemote), event -> {
            EValue.set(event.valueData.value.getBoolean());
        });
        instance.addListener(F, EnumSet.of(NetworkTableEvent.Kind.kValueRemote), event -> {
            FValue.set(event.valueData.value.getBoolean());
        });
        instance.addListener(G, EnumSet.of(NetworkTableEvent.Kind.kValueRemote), event -> {
            GValue.set(event.valueData.value.getBoolean());
        });
        instance.addListener(H, EnumSet.of(NetworkTableEvent.Kind.kValueRemote), event -> {
            HValue.set(event.valueData.value.getBoolean());
        });
        instance.addListener(I, EnumSet.of(NetworkTableEvent.Kind.kValueRemote), event -> {
            IValue.set(event.valueData.value.getBoolean());
        });
        instance.addListener(J, EnumSet.of(NetworkTableEvent.Kind.kValueRemote), event -> {
            JValue.set(event.valueData.value.getBoolean());
        });
        instance.addListener(K, EnumSet.of(NetworkTableEvent.Kind.kValueRemote), event -> {
            KValue.set(event.valueData.value.getBoolean());
        });
        instance.addListener(L, EnumSet.of(NetworkTableEvent.Kind.kValueRemote), event -> {
            LValue.set(event.valueData.value.getBoolean());
        });

        instance.addListener(L1, EnumSet.of(NetworkTableEvent.Kind.kValueRemote), event -> {
            L1Value.set(event.valueData.value.getBoolean());
        });
        instance.addListener(L2, EnumSet.of(NetworkTableEvent.Kind.kValueRemote), event -> {
            L2Value.set(event.valueData.value.getBoolean());
        });
        instance.addListener(L3, EnumSet.of(NetworkTableEvent.Kind.kValueRemote), event -> {
            L3Value.set(event.valueData.value.getBoolean());
        });
        instance.addListener(L4, EnumSet.of(NetworkTableEvent.Kind.kValueRemote), event -> {
            L4Value.set(event.valueData.value.getBoolean());
        });

        instance.addListener(knockOffAlgae, EnumSet.of(NetworkTableEvent.Kind.kValueRemote), event -> {
            knockOffAlgaeValue.set(event.valueData.value.getBoolean());
        });

        table.getBooleanTopic("A").publish();
        table.getBooleanTopic("B").publish();
        table.getBooleanTopic("C").publish();
        table.getBooleanTopic("D").publish();
        table.getBooleanTopic("E").publish();
        table.getBooleanTopic("F").publish();
        table.getBooleanTopic("G").publish();
        table.getBooleanTopic("H").publish();
        table.getBooleanTopic("I").publish();
        table.getBooleanTopic("J").publish();
        table.getBooleanTopic("K").publish();
        table.getBooleanTopic("L").publish();

        table.getBooleanTopic("L1").publish();
        table.getBooleanTopic("L2").publish();
        table.getBooleanTopic("L3").publish();
        table.getBooleanTopic("L4").publish();

        table.getBooleanTopic("Knock Off Algae").publish();
        
    }

    public void buttonBoxPeriodic() {
        if (AValue.getAndSet(false) != false) {
            selectedReef = ReefPosition.A;
            B.set(false);
            C.set(false);
            D.set(false);
            E.set(false);
            F.set(false);
            G.set(false);
            H.set(false);
            I.set(false);
            J.set(false);
            K.set(false);
            L.set(false);
        } else if (BValue.getAndSet(false) != false) {
            selectedReef = ReefPosition.B;
            A.set(false);
            C.set(false);
            D.set(false);
            E.set(false);
            F.set(false);
            G.set(false);
            H.set(false);
            I.set(false);
            J.set(false);
            K.set(false);
            L.set(false);
        } else if (CValue.getAndSet(false) != false) {
            selectedReef = ReefPosition.C;
            A.set(false);
            B.set(false);
            D.set(false);
            E.set(false);
            F.set(false);
            G.set(false);
            H.set(false);
            I.set(false);
            J.set(false);
            K.set(false);
            L.set(false);
        } else if (DValue.getAndSet(false) != false) {
            selectedReef = ReefPosition.D;
            A.set(false);
            B.set(false);
            C.set(false);
            E.set(false);
            F.set(false);
            G.set(false);
            H.set(false);
            I.set(false);
            J.set(false);
            K.set(false);
            L.set(false);
        } else if (EValue.getAndSet(false) != false) {
            selectedReef = ReefPosition.E;
            A.set(false);
            B.set(false);
            C.set(false);
            D.set(false);
            F.set(false);
            G.set(false);
            H.set(false);
            I.set(false);
            J.set(false);
            K.set(false);
            L.set(false);
        } else if (FValue.getAndSet(false) != false) {
            selectedReef = ReefPosition.F;
            A.set(false);
            B.set(false);
            C.set(false);
            D.set(false);
            E.set(false);
            G.set(false);
            H.set(false);
            I.set(false);
            J.set(false);
            K.set(false);
            L.set(false);
        } else if (GValue.getAndSet(false) != false) {
            selectedReef = ReefPosition.G;
            A.set(false);
            B.set(false);
            C.set(false);
            D.set(false);
            E.set(false);
            F.set(false);
            H.set(false);
            I.set(false);
            J.set(false);
            K.set(false);
            L.set(false);
        } else if (HValue.getAndSet(false) != false) {
            selectedReef = ReefPosition.H;
            A.set(false);
            B.set(false);
            C.set(false);
            D.set(false);
            E.set(false);
            F.set(false);
            G.set(false);
            I.set(false);
            J.set(false);
            K.set(false);
            L.set(false);
        } else if (IValue.getAndSet(false) != false) {
            selectedReef = ReefPosition.I;
            A.set(false);
            B.set(false);
            C.set(false);
            D.set(false);
            E.set(false);
            F.set(false);
            G.set(false);
            H.set(false);
            J.set(false);
            K.set(false);
            L.set(false);
        } else if (JValue.getAndSet(false) != false) {
            selectedReef = ReefPosition.J;
            A.set(false);
            B.set(false);
            C.set(false);
            D.set(false);
            E.set(false);
            F.set(false);
            G.set(false);
            H.set(false);
            I.set(false);
            K.set(false);
            L.set(false);
        } else if (KValue.getAndSet(false) != false) {
            selectedReef = ReefPosition.K;
            A.set(false);
            B.set(false);
            C.set(false);
            D.set(false);
            E.set(false);
            F.set(false);
            G.set(false);
            H.set(false);
            I.set(false);
            J.set(false);
            L.set(false);
        } else if (LValue.getAndSet(false) != false) {
            selectedReef = ReefPosition.L;
            A.set(false);
            B.set(false);
            C.set(false);
            D.set(false);
            E.set(false);
            F.set(false);
            G.set(false);
            H.set(false);
            I.set(false);
            J.set(false);
            K.set(false);
        }

        if (L1Value.getAndSet(false) != false) {
            selectedLevel = 1;
            L2.set(false);
            L3.set(false);
            L4.set(false);
        } else if (L2Value.getAndSet(false) != false) {
            selectedLevel = 2;
            L1.set(false);
            L3.set(false);
            L4.set(false);
        } else if (L3Value.getAndSet(false) != false) {
            selectedLevel = 3;
            L1.set(false);
            L2.set(false);
            L4.set(false);
        } else if (L4Value.getAndSet(false) != false) {
            selectedLevel = 4;
            L1.set(false);
            L2.set(false);
            L3.set(false);
        }

    }

    public static ReefPosition getSelectedReefBranch() {
        return selectedReef;
    } 

    public static int getSelectedReefLevel() {
        return selectedLevel;
    }

}
