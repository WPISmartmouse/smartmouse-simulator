#include <core/args.h>
#include <iostream>

int main(int argc, const char **argv) {
  args::ArgumentParser parser("Compiles a mouse (*.ms) file into cpp and h files.");

  args::HelpFlag help(parser, "help", "Display this help menu", {'h', "help"});
  args::Positional<std::string> mouse_filename(parser, "moue_filename", "*.ms file to compile",
                                               args::Options::Required);
  args::Positional<std::string> cpp_output_directory(parser, "cpp_output_dir", "directory to put the .cpp file in", ".");
  args::Positional<std::string> h_output_directory(parser, "h_output_dir", "directory to put the .h file in", ".");

  try {
    parser.ParseCLI(argc, argv);
  }
  catch (args::Help &e) {
    std::cerr << parser;
    return 0;
  }
  catch (args::RequiredError &e) {
    std::cerr << parser;
    return 0;
  }

  std::cout << "Compiling " << args::get(mouse_filename) << '\n';
  std::cout << "Output cpp file: " << args::get(cpp_output_directory) << '\n';
  std::cout << "Output h file: " << args::get(h_output_directory) << '\n';
}
