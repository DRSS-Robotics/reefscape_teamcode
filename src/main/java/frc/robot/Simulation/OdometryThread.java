// package frc.robot.Simulation;

// import static frc.robot.generated.TunerConstants.*;

// import java.util.ArrayList;
// import java.util.List;
// import java.util.Queue;
// import java.util.concurrent.ArrayBlockingQueue;
// import java.util.function.Supplier;

// import com.ctre.phoenix6.BaseStatusSignal;
// import com.ctre.phoenix6.StatusSignal;

// import frc.robot.generated.TunerConstants;

// public interface OdometryThread {
//     class OdemtryInputs<T> {
//         private final Supplier<T> supplier;
//         private final Queue<T> queue;
        

//         public OdemtryInputs(Supplier<T> signal) {
//             this.supplier = signal;
//             this.queue = new ArrayBlockingQueue<>(TunerConstants.ODOEMTRY_CACHE_CAPACITY);
//         }

//         public void cacheInputToQueue() {
//             this.queue.offer(supplier.get());
//         }

//     }

//     List<OdemtryInputs> registerInputs = new ArrayList<>();
//     List<BaseStatusSignal> registeredStatusSignals = new ArrayList<>();

//     static <T>  Queue<T> registerSignalSignal(StatusSignal<T> signal) {
//         registeredStatusSignals.add(signal);
//         return registerInput(signal.asSupplier());
//     }

//     static <T> Queue<T> registerInput(Supplier<T> supplier) {
//         final OdemtryInputs odemtryInput = new OdemtryInputs<>(supplier);
//         registerInputs.add(odemtryInput);
//         return odemtryInput.queue;
//     }

// //gotta add logging later
// }
