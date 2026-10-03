# The one `nixpkgs` instantiation every other module in this repo receives.
#
# It is a module rather than a `let` binding so that every output — dev shells,
# the host package, the formatter app — is built from the same, single
# instantiation with the same overlays.
{
  nixpkgs,
  system,
  rust-overlay,
}:
import nixpkgs {
  inherit system;
  overlays = [ (import rust-overlay) ];
}
