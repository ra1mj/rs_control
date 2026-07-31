use hsm_spike::{StatigMachine, workload_checksum};

fn main() {
    println!("{}", workload_checksum::<StatigMachine>());
}
