package edu.wpi.first.wpilibj;

import java.io.File;

public final class Filesystem {
  private Filesystem() {}

  public static File getDeployDirectory() {
    return new File(System.getProperty("deploy.dir", "src/main/deploy"));
  }
}
