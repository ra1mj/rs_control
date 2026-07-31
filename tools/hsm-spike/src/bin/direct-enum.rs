use hsm_spike::{DirectMachine, workload_checksum};

fn main() {
    println!("{}", workload_checksum::<DirectMachine>());
}
