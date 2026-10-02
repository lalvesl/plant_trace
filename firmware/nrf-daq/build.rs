use std::{env, fs::File, io::Write, path::PathBuf};

fn main() {
    // `cortex-m-rt`'s linker script includes `memory.x` from the link search
    // path, so it has to be copied where the linker will look.
    let out = PathBuf::from(env::var("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=build.rs");
}
