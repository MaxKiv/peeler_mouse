fn main() {
    embuild::espidf::sysenv::output();
    embuild::build::CfgArgs::output_propagated("ESP_IDF").unwrap();
    embuild::build::LinkArgs::output_propagated("ESP_IDF").unwrap();
}
