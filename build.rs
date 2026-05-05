fn main() {
    cc::Build::new()
        .file("clib/src/library.c")
        .include("clib/include")
        .compile("mag_math"); // Links as libmag_math.a

    // Tell cargo to rebuild if the C files change
    println!("cargo:rerun-if-changed=clib/src/library.c");
    println!("cargo:rerun-if-changed=clib/include/library.h");
}
