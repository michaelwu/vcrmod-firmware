use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("mkl26z32.ld")).unwrap()
        .write_all(include_bytes!("mkl26z32.ld")).unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun_if_changed=mkl26z32.ld");
}
