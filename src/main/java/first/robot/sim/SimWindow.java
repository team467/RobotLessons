package first.robot.sim;

import static first.robot.constants.FieldConstants.*;
import static first.robot.constants.RobotConstants.*;

import first.robot.constants.DriverConstants;
import first.robot.mechanisms.drive.DriveConstants;
import java.awt.BasicStroke;
import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.FlowLayout;
import java.awt.Font;
import java.awt.FontMetrics;
import java.awt.GradientPaint;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.GraphicsConfiguration;
import java.awt.GraphicsEnvironment;
import java.awt.Image;
import java.awt.Insets;
import java.awt.Rectangle;
import java.awt.RadialGradientPaint;
import java.awt.RenderingHints;
import java.awt.Toolkit;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Line2D;
import java.awt.geom.Path2D;
import java.awt.geom.Rectangle2D;
import java.awt.geom.RoundRectangle2D;
import java.net.URL;
import javax.swing.ImageIcon;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.SwingUtilities;
import javax.swing.Timer;

/**
 * Interactive 2D simulation animation displaying the side elevation view of the robot and field.
 *
 * <p>The camera horizontally tracks with the robot (centering on the robot's drive position),
 * rendering the mechanism (drivetrain, elevator mast/carriage, telescoping extender, and claw)
 * as it interacts with the field environment: start zone, game pieces (Green Ball 1 and Orange Ball 2),
 * table obstacle with collision detection, and target receptacles.
 */
public class SimWindow extends JFrame {
  // Visible area of the side-view elevation world, in meters.
  // The camera tracks the robot horizontally with forward lookahead so the table and basket are visible.
  private static final double VIEW_SPAN_X = 2.80;
  private static final double VIEW_HALF_X = VIEW_SPAN_X / 2.0;
  private static final double VIEW_MIN_Y = -0.38;
  private static final double VIEW_MAX_Y = 1.95;

  private static final Color SKY_TOP = new Color(0xDCE6F0);
  private static final Color SKY_BOTTOM = new Color(0xF7F9FB);
  private static final Color GRID = new Color(0, 0, 0, 18);
  private static final Color FLOOR = new Color(0x5B6470);
  private static final Color TARGET_GREEN = new Color(0x2E9E4F);
  private static final Color TARGET_GREEN_LIT = new Color(0x4CFF7A);
  private static final Color TARGET_ORANGE = new Color(0xEA580C);
  private static final Color TARGET_ORANGE_LIT = new Color(0xFBA04B);
  private static final Color TARGET_BASKET = new Color(0x7C3AED);
  private static final Color TARGET_BASKET_LIT = new Color(0xA78BFA);
  private static final Color TABLE_TOP = new Color(0x8B5A2B);
  private static final Color TABLE_LEG = new Color(0x6E4520);
  private static final Color ALUMINUM = new Color(0xB8C0C8);
  private static final Color ALUMINUM_DARK = new Color(0x8A939C);
  private static final Color ALUMINUM_LIGHT = new Color(0xD8DEE4);
  private static final Color MOTOR = new Color(0x2B2F33);
  private static final Color MOTOR_RUNNING = new Color(0x22C55E); // Green when actively moving
  private static final Color MOTOR_STALLED = new Color(0xEF4444); // Red when stalled at limit
  private static final Color CARRIAGE = new Color(0x3C6E9E);
  private static final Color CHAIN = new Color(0x444444);
  private static final Color FINGER = new Color(0x3A3F44);
  private static final Color SETPOINT = new Color(0xE53935);
  private static final Color HUD_BG = new Color(20, 24, 28, 210);
  private static final Color WHEEL_TIRE = new Color(0x1E2022);
  private static final Color WHEEL_RIM = new Color(0x717D8A);
  private static final Color BUMPER_BLUE = new Color(0x1D4ED8); // FRC Team 467 royal blue
  private static final Color BUMPER_BLUE_DARK = new Color(0x172554);

  private static Color motorColor(SimWorld.MotorState state) {
    if (state == null) {
      return MOTOR;
    }
    return switch (state) {
      case RUNNING -> MOTOR_RUNNING;
      case STALLED -> MOTOR_STALLED;
      default -> MOTOR;
    };
  }

  private final SimWorld world;
  private final ScenePanel scene = new ScenePanel();
  private boolean resetKeyWasDown = false;
  private boolean teleopKeyWasDown = false;

  /** Opens the window (does nothing when running without a display, e.g. in CI). */
  public static void launch(SimWorld world, SimKeyboard keyboard) {
    if (GraphicsEnvironment.isHeadless()) {
      return;
    }
    SwingUtilities.invokeLater(() -> new SimWindow(world, keyboard).setVisible(true));
  }

  private SimWindow(SimWorld world, SimKeyboard keyboard) {
    super("467 Sim");
    loadWindowIcon();
    this.world = world;
    keyboard.install();

    setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
    setLayout(new BorderLayout());
    add(scene, BorderLayout.CENTER);

    JButton resetBtn = new JButton("Reset State (R)");
    resetBtn.setFocusable(false);
    resetBtn.addActionListener(e -> world.requestResetState());

    JButton teleopBtn = new JButton("Teleop (T)");
    teleopBtn.setFocusable(false);
    teleopBtn.addActionListener(e -> world.requestTeleop());

    JButton shootBallsBtn = new JButton("Auto: Shoot Balls");
    shootBallsBtn.setFocusable(false);
    shootBallsBtn.addActionListener(e -> world.requestAutoShootBalls());

    JButton twoBallBtn = new JButton("Auto: Two Ball");
    twoBallBtn.setFocusable(false);
    twoBallBtn.addActionListener(e -> world.requestAutoTwoBall());

    JButton scoreBtn = new JButton("Auto: Score Ball");
    scoreBtn.setFocusable(false);
    scoreBtn.addActionListener(e -> world.requestAutoScore());

    JButton disableBtn = new JButton("Disable");
    disableBtn.setFocusable(false);
    disableBtn.addActionListener(e -> world.requestDisable());

    JPanel buttons = new JPanel(new FlowLayout(FlowLayout.CENTER, 8, 6));
    buttons.add(resetBtn);
    buttons.add(teleopBtn);
    buttons.add(shootBallsBtn);
    buttons.add(twoBallBtn);
    buttons.add(scoreBtn);
    buttons.add(disableBtn);
    add(buttons, BorderLayout.SOUTH);

    addWindowFocusListener(
        new WindowAdapter() {
          @Override
          public void windowLostFocus(WindowEvent e) {
            keyboard.releaseAll();
          }
        });

    pack();
    positionHigherOnScreen();

    new Timer(
            20,
            e -> {
              boolean resetDown = keyboard.isPressed(DriverConstants.KEY_RESET_STATE);
              if (resetDown && !resetKeyWasDown) {
                world.requestResetState();
              }
              resetKeyWasDown = resetDown;

              boolean teleopDown = keyboard.isPressed(DriverConstants.KEY_TELEOP);
              if (teleopDown && !teleopKeyWasDown) {
                world.requestTeleop();
              }
              teleopKeyWasDown = teleopDown;

              scene.repaint();
            })
        .start();
  }
 
  /**
   * Positions the window towards the top of the usable screen area so that the bottom
   * controls and key legend are not cut off by the taskbar or display edge.
   */
  private void positionHigherOnScreen() {
    try {
      GraphicsConfiguration gc = getGraphicsConfiguration();
      Rectangle screenBounds;
      Insets screenInsets;
      if (gc != null) {
        screenBounds = gc.getBounds();
        screenInsets = Toolkit.getDefaultToolkit().getScreenInsets(gc);
      } else {
        screenBounds = new Rectangle(Toolkit.getDefaultToolkit().getScreenSize());
        screenInsets = new Insets(0, 0, 0, 0);
      }

      int usableX = screenBounds.x + screenInsets.left;
      int usableY = screenBounds.y + screenInsets.top;
      int usableW = screenBounds.width - screenInsets.left - screenInsets.right;
      int usableH = screenBounds.height - screenInsets.top - screenInsets.bottom;

      int winW = getWidth();
      int winH = getHeight();

      // If window exceeds usable height, scale it down to fit within usable area
      if (winH > usableH) {
        winH = usableH;
        setSize(winW, winH);
      }

      // Center horizontally on the screen
      int posX = usableX + Math.max(0, (usableW - winW) / 2);
      // Place near the top with a small margin (e.g. 20px) to ensure bottom controls are visible
      int topMargin = Math.min(20, Math.max(0, (usableH - winH) / 4));
      int posY = usableY + topMargin;

      setLocation(posX, posY);
    } catch (Exception e) {
      setLocationByPlatform(true);
    }
  }

  private void loadWindowIcon() {
    try {
      URL iconUrl = getClass().getResource("/icon.png");
      if (iconUrl != null) {
        Image img = new ImageIcon(iconUrl).getImage();
        setIconImage(img);
        return;
      }

      // Fallback: check file on filesystem
      java.io.File file = new java.io.File("src/main/resources/icon.png");
      if (file.exists()) {
        Image img = new ImageIcon(file.getAbsolutePath()).getImage();
        setIconImage(img);
      }
    } catch (Exception ignored) {
    }
  }

  private class ScenePanel extends JPanel {
    private double sideScale;
    private double sideOriginX;
    private double sideOriginY;
    private double cameraCenterX = 0.0;

    ScenePanel() {
      setPreferredSize(new Dimension(1060, 720));
      setFocusable(true);
    }

    // ---- Side View Coordinates: world meters (Field X) -> pixels ----
    // Robot is always kept in the horizontal center of the lower pane
    private double sx(double fieldX) {
      return sideOriginX + ((fieldX - cameraCenterX) + VIEW_HALF_X) * sideScale;
    }

    private double sy(double y) {
      return sideOriginY + (VIEW_MAX_Y - y) * sideScale;
    }

    private double len(double meters) {
      return meters * sideScale;
    }

    private Rectangle2D rect(double x1, double y1, double x2, double y2) {
      return new Rectangle2D.Double(
          sx(Math.min(x1, x2)),
          sy(Math.max(y1, y2)),
          len(Math.abs(x2 - x1)),
          len(Math.abs(y2 - y1)));
    }

    private RoundRectangle2D roundRect(double x1, double y1, double x2, double y2, double r) {
      Rectangle2D b = rect(x1, y1, x2, y2);
      return new RoundRectangle2D.Double(
          b.getX(), b.getY(), b.getWidth(), b.getHeight(), len(r), len(r));
    }

    private void fillOutlined(Graphics2D g, java.awt.Shape s, Color fill, Color outline) {
      g.setColor(fill);
      g.fill(s);
      g.setColor(outline);
      g.setStroke(new BasicStroke(1.2f));
      g.draw(s);
    }

    @Override
    protected void paintComponent(Graphics graphics) {
      super.paintComponent(graphics);
      Graphics2D g = (Graphics2D) graphics.create();
      g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
      g.setRenderingHint(
          RenderingHints.KEY_TEXT_ANTIALIASING, RenderingHints.VALUE_TEXT_ANTIALIAS_ON);

      int w = getWidth();
      int h = getHeight();

      SimWorld.Snapshot s = world.snapshot();

      // Full-window Side Elevation View with camera centered ahead of the robot so the table and basket fit in view
      sideScale = Math.min(w / VIEW_SPAN_X, h / (VIEW_MAX_Y - VIEW_MIN_Y));
      sideOriginX = (w - VIEW_SPAN_X * sideScale) / 2.0;
      sideOriginY = (h - (VIEW_MAX_Y - VIEW_MIN_Y) * sideScale) / 2.0;
      cameraCenterX = s.drivePosition() + 0.65;

      double robFieldX = s.drivePosition();
      double wristFieldX = robFieldX + WRIST_BASE_X + s.extenderPosition();
      double wristY = s.elevatorHeight();

      // Background, scrolling floor, scrolling start zone, targets and table
      drawSideBackground(g, w, h, cameraCenterX);
      drawStartZoneSide(g);
      drawFloorAndTargets(g, s.scored() && !s.basketScored(), s.ball2Scored() && !s.basketScored(), s.basketScored());
      drawTable(g, s.tableContact());

      // Robot is always in the side view
      drawWheelsSide(g, robFieldX);
      drawTower(g, robFieldX, s);
      drawBumperSide(g, robFieldX);
      drawCarriageAndExtender(g, robFieldX, wristY, wristFieldX, s);
      drawShooter(g, robFieldX, s);
      drawClaw(g, wristFieldX, wristY, s.clawAngle(), s);
      drawSetpointMarkers(g, robFieldX, s, wristY);

      // Ball 1 (Green ball)
      drawBall(g, s.ball1FieldX(), s.ballY(), false);

      // Ball 2 (Orange ball)
      drawBall(g, s.ball2FieldX(), s.ball2SideY(), true);

      drawHud(g, s, w, h);
      g.dispose();
    }

    // ==========================================================================================
    // SIDE ELEVATION VIEW (Robot-Centered, Scrolling Background)
    // ==========================================================================================

    private void drawSideBackground(Graphics2D g, int w, int h, double camX) {
      g.setPaint(new GradientPaint(0, 0, SKY_TOP, 0, h, SKY_BOTTOM));
      g.fillRect(0, 0, w, h);

      // Scrolling grid lines in world coordinates
      g.setColor(GRID);
      g.setStroke(new BasicStroke(1f));
      double startGridX = Math.floor((camX - VIEW_HALF_X) / 0.25) * 0.25;
      double endGridX = Math.ceil((camX + VIEW_HALF_X) / 0.25) * 0.25;
      for (double gx = startGridX; gx <= endGridX; gx += 0.25) {
        double px = sx(gx);
        g.draw(new Line2D.Double(px, sy(VIEW_MAX_Y), px, sy(0)));
      }
      for (double y = 0.25; y <= VIEW_MAX_Y; y += 0.25) {
        g.draw(new Line2D.Double(0, sy(y), w, sy(y)));
      }
    }

    private void drawStartZoneSide(Graphics2D g) {
      double z1 = ROBOT_START_X - 0.40;
      double z2 = ROBOT_START_X + 0.40;
      g.setColor(new Color(59, 130, 246, 35));
      g.fill(rect(z1, 0.0, z2, 0.015));
      g.setColor(new Color(96, 165, 250, 180));
      g.setStroke(
          new BasicStroke(
              1.5f, BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER, 10f, new float[] {4f, 4f}, 0f));
      g.draw(new Line2D.Double(sx(z1), sy(0), sx(z1), sy(0.40)));
      g.draw(new Line2D.Double(sx(z2), sy(0), sx(z2), sy(0.40)));
      g.setFont(new Font(Font.SANS_SERIF, Font.BOLD, 10));
      g.setColor(new Color(59, 130, 246));
      drawCentered(g, "START ZONE", sx(ROBOT_START_X), sy(-0.045));
    }

    private void drawFloorAndTargets(
        Graphics2D g, boolean scoredGreen, boolean scoredOrange, boolean scoredBasket) {
      // Continuous scrolling floor
      g.setColor(FLOOR);
      g.fillRect(0, (int) sy(0.0), getWidth(), getHeight() - (int) sy(0.0));

      // 1. Green Floor Target Box near Table
      double tgtLeft = ROBOT_TABLE_X + TARGET_LEFT_X;
      double tgtRight = ROBOT_TABLE_X + TARGET_RIGHT_X;
      drawTargetBox(g, tgtLeft, tgtRight, 0.0, TARGET_BOX_WALL_HEIGHT, TARGET_GREEN, TARGET_GREEN_LIT, scoredGreen, "TARGET (Grn)");

      // 2. Orange Table Target Box on top of the Table
      double tblTgtLeft = ROBOT_TABLE_X + TABLE_TARGET_LEFT_X;
      double tblTgtRight = ROBOT_TABLE_X + TABLE_TARGET_RIGHT_X;
      drawTargetBox(g, tblTgtLeft, tblTgtRight, TABLE_HEIGHT, TABLE_HEIGHT + TARGET_BOX_WALL_HEIGHT, TARGET_ORANGE, TARGET_ORANGE_LIT, scoredOrange, "TARGET (Org)");

      // 3. Elevated Basket in the air to the right of the Table
      double bskLeft = ROBOT_TABLE_X + BASKET_LEFT_X;
      double bskRight = ROBOT_TABLE_X + BASKET_RIGHT_X;
      // Vertical support post, diagonal brace, and backboard holding the basket in the air
      double postX = bskRight + 0.02;
      fillOutlined(g, rect(postX, 0.0, postX + 0.035, BASKET_HEIGHT + 0.24), ALUMINUM_DARK, Color.DARK_GRAY);
      fillOutlined(g, roundRect(bskRight + 0.005, BASKET_HEIGHT - 0.02, postX + 0.045, BASKET_HEIGHT + 0.28, 0.01), ALUMINUM_LIGHT, ALUMINUM_DARK);
      drawTargetBox(
          g,
          bskLeft,
          bskRight,
          BASKET_HEIGHT,
          BASKET_HEIGHT + TARGET_BOX_WALL_HEIGHT + 0.03,
          TARGET_BASKET,
          TARGET_BASKET_LIT,
          scoredBasket,
          "BASKET");

      // If the robot is back near the start zone and the basket is off the right edge of the view,
      // show a directional indicator badge at the right edge of the screen.
      if (sx(bskLeft) > getWidth() - 20) {
        int badgeW = 148;
        int badgeH = 28;
        int bx = getWidth() - badgeW - 14;
        int by = (int) sy(BASKET_HEIGHT + 0.05) - badgeH / 2;
        g.setColor(new Color(124, 58, 237, 220));
        g.fillRoundRect(bx, by, badgeW, badgeH, 10, 10);
        g.setColor(Color.WHITE);
        g.setFont(new Font(Font.SANS_SERIF, Font.BOLD, 12));
        drawCentered(g, "TABLE & BASKET ->", bx + badgeW / 2.0, by + 18);
      }
    }

    /**
     * Draws an open container/box that is 1/3 the height of the ball,
     * with bottom plate and side walls rising upward.
     */
    private void drawTargetBox(
        Graphics2D g,
        double x1,
        double x2,
        double yBase,
        double yTop,
        Color baseCol,
        Color litCol,
        boolean isScored,
        String label) {
      Color mainCol = isScored ? litCol : baseCol;
      double wallThickness = 0.020;
      // Interior glowing receptacle
      g.setColor(new Color(mainCol.getRed(), mainCol.getGreen(), mainCol.getBlue(), isScored ? 120 : 50));
      g.fill(rect(x1, yBase, x2, yTop));

      // Bottom base plate
      fillOutlined(g, rect(x1, yBase, x2, yBase + 0.015), mainCol, mainCol.darker());

      // Left vertical wall (height = 1/3 ball height)
      fillOutlined(g, roundRect(x1 - wallThickness, yBase, x1, yTop, 0.005), mainCol, mainCol.darker());

      // Right vertical wall (height = 1/3 ball height)
      fillOutlined(g, roundRect(x2, yBase, x2 + wallThickness, yTop, 0.005), mainCol, mainCol.darker());

      // Top opening dashed indicator
      g.setColor(isScored ? litCol : mainCol.brighter());
      g.setStroke(
          new BasicStroke(
              1.5f, BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER, 10f, new float[] {4f, 3f}, 0f));
      g.draw(new Line2D.Double(sx(x1), sy(yTop), sx(x2), sy(yTop)));

      // Label below box or on surface
      g.setFont(new Font(Font.SANS_SERIF, Font.BOLD, (int) Math.max(10, len(0.040))));
      g.setColor(mainCol.darker());
      drawCentered(g, label, sx((x1 + x2) / 2.0), sy(yBase - 0.025));
    }

    private void drawTable(Graphics2D g, boolean contact) {
      double tblLeft = ROBOT_TABLE_X + TABLE_LEFT_X;
      double tblRight = ROBOT_TABLE_X + TABLE_RIGHT_X;
      double legW = 0.04;
      double topThickness = 0.045;
      Color topColor = contact ? new Color(0xDC2626) : TABLE_TOP;
      Color legColor = contact ? new Color(0x991B1B) : TABLE_LEG;

      fillOutlined(
          g, rect(tblLeft + 0.03, 0, tblLeft + 0.03 + legW, TABLE_HEIGHT), legColor,
          legColor.darker());
      fillOutlined(
          g, rect(tblRight - 0.03 - legW, 0, tblRight - 0.03, TABLE_HEIGHT), legColor,
          legColor.darker());
      fillOutlined(
          g,
          roundRect(tblLeft, TABLE_HEIGHT - topThickness, tblRight, TABLE_HEIGHT, 0.01),
          topColor,
          topColor.darker());

      if (contact) {
        g.setFont(new Font(Font.SANS_SERIF, Font.BOLD, (int) Math.max(12, len(0.038))));
        g.setColor(Color.WHITE);
        drawCentered(g, "HIT TABLE!", sx((tblLeft + tblRight) / 2.0), sy(TABLE_HEIGHT - 0.014));
      }
    }

    private void drawWheelsSide(Graphics2D g, double rx) {
      double wr = DriveConstants.WHEEL_RADIUS_METERS;
      double[] wheelCentersX = {rx - 0.20, rx + 0.14};

      for (double wx : wheelCentersX) {
        Ellipse2D tire = new Ellipse2D.Double(sx(wx - wr), sy(2 * wr), len(2 * wr), len(2 * wr));
        fillOutlined(g, tire, WHEEL_TIRE, Color.BLACK);

        double rimR = wr * 0.65;
        Ellipse2D rim = new Ellipse2D.Double(sx(wx - rimR), sy(wr + rimR), len(2 * rimR), len(2 * rimR));
        fillOutlined(g, rim, WHEEL_RIM, ALUMINUM_DARK);

        double hubR = wr * 0.22;
        Ellipse2D hub = new Ellipse2D.Double(sx(wx - hubR), sy(wr + hubR), len(2 * hubR), len(2 * hubR));
        fillOutlined(g, hub, ALUMINUM_LIGHT, Color.BLACK);
      }
    }

    private void drawBumperSide(Graphics2D g, double rx) {
      double bLeft = rx - 0.35;
      double bRight = rx + 0.35;
      double bBottom = 0.045;
      double bTop = 0.165;

      // Outer blue bumper perimeter (FRC standard blue fabric)
      RoundRectangle2D bumperBody = roundRect(bLeft, bBottom, bRight, bTop, 0.015);
      fillOutlined(g, bumperBody, BUMPER_BLUE, BUMPER_BLUE_DARK);

      // Fabric highlight and bevel line
      g.setColor(new Color(255, 255, 255, 60));
      g.setStroke(new BasicStroke(1.2f));
      g.draw(new Line2D.Double(sx(bLeft + 0.01), sy(bTop - 0.01), sx(bRight - 0.01), sy(bTop - 0.01)));

      // White "467" team number on the bumper: italic, right-aligned, and lower
      int fontSize = (int) Math.max(14, len(0.082));
      g.setFont(new Font(Font.SANS_SERIF, Font.BOLD | Font.ITALIC, fontSize));
      FontMetrics fm = g.getFontMetrics();
      int textW = fm.stringWidth("467");

      // Placed a little lower on the bumper
      double baselineY = sy(bBottom + 0.015);
      // Right-aligned on the side bumper with ~0.04m margin from right edge
      double rightPixelX = sx(bRight - 0.04);
      double textPixelX = rightPixelX - textW;

      // Drop shadow for embroidery effect
      g.setColor(new Color(0, 0, 0, 160));
      g.drawString("467", (float) (textPixelX + 1.5), (float) (baselineY + 1.5));
      // Crisp white numbers
      g.setColor(Color.WHITE);
      g.drawString("467", (float) textPixelX, (float) baselineY);
    }

    private void drawTower(Graphics2D g, double rx, SimWorld.Snapshot s) {
      // Chassis base plate and elevator motor
      fillOutlined(g, rect(rx - 0.30, 0.045, rx + 0.22, 0.09), ALUMINUM_DARK, ALUMINUM_DARK.darker());
      fillOutlined(g, roundRect(rx - 0.27, 0.09, rx - 0.11, 0.20, 0.02), motorColor(s.elevatorMotorState()), Color.BLACK);
      label(g, "Y motor", rx - 0.19, 0.23);

      // Drive motor balancing on chassis plate
      fillOutlined(g, roundRect(rx + 0.09, 0.09, rx + 0.21, 0.18, 0.02), motorColor(s.driveMotorState()), Color.BLACK);
      label(g, "Drive motor", rx + 0.15, 0.21);

      // Two vertical rails
      fillOutlined(g, rect(rx - 0.075, 0.09, rx - 0.04, TOWER_HEIGHT), ALUMINUM, ALUMINUM_DARK);
      fillOutlined(g, rect(rx + 0.04, 0.09, rx + 0.075, TOWER_HEIGHT), ALUMINUM, ALUMINUM_DARK);
      fillOutlined(g, rect(rx - 0.09, TOWER_HEIGHT - 0.03, rx + 0.09, TOWER_HEIGHT), ALUMINUM_DARK,
          ALUMINUM_DARK.darker());

      // Chain & sprockets
      double sprocketR = 0.028;
      double bottomY = 0.15;
      double topY = TOWER_HEIGHT - 0.06;
      g.setColor(CHAIN);
      g.setStroke(
          new BasicStroke(
              2f, BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER, 10f, new float[] {4f, 2f}, 0f));
      g.draw(new Line2D.Double(sx(rx - sprocketR), sy(bottomY), sx(rx - sprocketR), sy(topY)));
      g.draw(new Line2D.Double(sx(rx + sprocketR), sy(bottomY), sx(rx + sprocketR), sy(topY)));
      for (double y : new double[] {bottomY, topY}) {
        Ellipse2D sprocket =
            new Ellipse2D.Double(sx(rx - sprocketR), sy(y + sprocketR), len(2 * sprocketR),
                len(2 * sprocketR));
        fillOutlined(g, sprocket, ALUMINUM_DARK, CHAIN);
      }
    }

    /**
     * Draws the spring-loaded shooter mounted on the front of the robot tower:
     * mounting bracket, pullback motor, ratchet wheel & pawl, release solenoid, coil spring, and kicker foot.
     * Positioned so its extended kicker strikes a ball held near elevator=0.60m, extender=0.23m (ball center ~ rx+0.35, y=0.46),
     * while balls held too close to the robot (extender < 0.17m) sit behind the kicker.
     */
    private void drawShooter(Graphics2D g, double rx, SimWorld.Snapshot s) {
      double baseX = rx + 0.09;
      double baseY = 0.31;
      double pullback = s.shooterPullback(); // 0.0 (extended) to 0.25 (pulled back)
      double extension = Math.max(0.03, 0.19 - pullback * 0.58);

      // Mounting bracket attached to front of tower
      fillOutlined(g, rect(rx + 0.055, baseY - 0.01, baseX + 0.125, baseY + 0.095), ALUMINUM_DARK, Color.DARK_GRAY);

      // Pullback winch motor housing (colored green when running, red when stalled)
      fillOutlined(g, roundRect(baseX, baseY, baseX + 0.12, baseY + 0.075, 0.015), motorColor(s.shooterMotorState()), Color.BLACK);
      label(g, "Shooter", baseX + 0.06, baseY - 0.025);

      // Ratchet wheel on the shooter shaft
      double gearCx = baseX + 0.06;
      double gearCy = baseY + 0.038;
      double gearR = 0.022;
      Ellipse2D ratchetWheel =
          new Ellipse2D.Double(sx(gearCx - gearR), sy(gearCy + gearR), len(2 * gearR), len(2 * gearR));
      fillOutlined(
          g,
          ratchetWheel,
          s.shooterRatchetLocked() ? new Color(0xFBBF24) : ALUMINUM_LIGHT,
          Color.BLACK);

      // Solenoid cylinder & ratchet locking pawl on top of shooter housing
      Color solenoidCol =
          s.shooterSolenoidReleased()
              ? new Color(0x38BDF8) // Bright cyan when solenoid releases ratchet
              : (s.shooterLoaded() ? new Color(0xF59E0B) : new Color(0x64748B));
      fillOutlined(g, roundRect(baseX + 0.01, baseY + 0.08, baseX + 0.11, baseY + 0.12, 0.01), solenoidCol, Color.BLACK);
      label(g, s.shooterSolenoidReleased() ? "Solenoid: OPEN" : "Ratchet LOCK", baseX + 0.06, baseY + 0.13);

      // Coil spring extending from front of shooter housing toward the kicker plate (aimed at ball when elevator=0.60, extender=0.23)
      double startX = baseX + 0.12;
      double startY = baseY + 0.045;
      double tipX = startX + extension;
      double tipY = startY + extension * 0.45;

      g.setColor(s.shooterLoaded() ? new Color(0xF59E0B) : new Color(0xEA580C));
      g.setStroke(new BasicStroke((float) Math.max(2.5, len(0.010))));
      int coils = 6;
      Path2D spring = new Path2D.Double();
      spring.moveTo(sx(startX), sy(startY));
      for (int i = 1; i <= coils; i++) {
        double t = (double) i / (coils + 1);
        double side = (i % 2 == 0) ? 0.020 : -0.020;
        double cx = startX + (tipX - startX) * t - side * 0.4;
        double cy = startY + (tipY - startY) * t + side;
        spring.lineTo(sx(cx), sy(cy));
      }
      spring.lineTo(sx(tipX), sy(tipY));
      g.draw(spring);

      // Heavy curved kicker plate at the end of the spring
      Path2D kicker = new Path2D.Double();
      kicker.moveTo(sx(tipX - 0.014), sy(tipY + 0.036));
      kicker.lineTo(sx(tipX + 0.018), sy(tipY - 0.028));
      g.setColor(new Color(0xF97316));
      g.setStroke(new BasicStroke((float) Math.max(4.0, len(0.018)), BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
      g.draw(kicker);
    }

    private void drawCarriageAndExtender(Graphics2D g, double rx, double h, double wristX, SimWorld.Snapshot s) {
      // Carriage
      fillOutlined(g, roundRect(rx - 0.10, h - 0.09, rx + 0.11, h + 0.06, 0.015), CARRIAGE,
          CARRIAGE.darker());

      // Inner moving stage and outer fixed stage
      double innerStart = Math.max(rx + 0.02, wristX - 0.62);
      fillOutlined(g, rect(innerStart, h + 0.008, wristX, h + 0.042), ALUMINUM_LIGHT,
          ALUMINUM_DARK);
      fillOutlined(g, rect(rx, h - 0.005, rx + 0.64, h + 0.055), new Color(184, 192, 200, 210),
          ALUMINUM_DARK);

      // Pulley belt
      g.setColor(CHAIN);
      g.setStroke(new BasicStroke((float) Math.max(1.5, len(0.006))));
      g.draw(new Line2D.Double(sx(rx + 0.06), sy(h + 0.025), sx(rx + 0.62), sy(h + 0.025)));

      // Extender motor
      fillOutlined(g, roundRect(rx, h + 0.055, rx + 0.13, h + 0.14, 0.02), motorColor(s.extenderMotorState()), Color.BLACK);
      label(g, "X motor", rx + 0.065, h + 0.17);
    }

    private void drawBall(Graphics2D g, double x, double y, boolean isOrange) {
      double r = BALL_RADIUS;
      Ellipse2D ball = new Ellipse2D.Double(sx(x - r), sy(y + r), len(2 * r), len(2 * r));

      Color c1 = isOrange ? new Color(0xFED7AA) : new Color(0xBBF7D0);
      Color c2 = isOrange ? new Color(0xF97316) : new Color(0x22C55E);
      Color c3 = isOrange ? new Color(0xC2410C) : new Color(0x15803D);
      Color outline = isOrange ? new Color(0x9A3412) : new Color(0x166534);

      g.setPaint(
          new RadialGradientPaint(
              (float) sx(x - r * 0.35),
              (float) sy(y + r * 0.35),
              (float) len(r * 1.4),
              new float[] {0f, 0.6f, 1f},
              new Color[] {c1, c2, c3}));
      g.fill(ball);
      g.setColor(outline);
      g.setStroke(new BasicStroke(1.5f));
      g.draw(ball);

      // Ball seam
      g.draw(
          new Ellipse2D.Double(
              sx(x - r * 0.35), sy(y + r), len(r * 0.7), len(2 * r)));
    }

    private void drawClaw(Graphics2D g, double wristX, double wristY, double angle, SimWorld.Snapshot s) {
      fillOutlined(g, roundRect(wristX - 0.06, wristY - 0.03, wristX + 0.06, wristY + 0.06, 0.01),
          ALUMINUM_DARK, ALUMINUM_DARK.darker());
      fillOutlined(g, roundRect(wristX - 0.028, wristY + 0.06, wristX + 0.028, wristY + 0.13,
          0.01), motorColor(s.clawMotorState()), Color.BLACK);

      for (int side : new int[] {-1, 1}) {
        double pivotX = wristX + side * FINGER_PIVOT_HALF_SPACING;
        double pivotY = wristY - 0.02;
        double dirX = side * Math.sin(angle);
        double dirY = -Math.cos(angle);
        double tipX = pivotX + dirX * FINGER_LENGTH;
        double tipY = pivotY + dirY * FINGER_LENGTH;
        double hookX = tipX - side * Math.cos(angle) * 0.035;
        double hookY = tipY - Math.sin(angle) * 0.035;

        Path2D finger = new Path2D.Double();
        finger.moveTo(sx(pivotX), sy(pivotY));
        finger.lineTo(sx(tipX), sy(tipY));
        finger.lineTo(sx(hookX), sy(hookY));

        g.setColor(Color.BLACK);
        g.setStroke(
            new BasicStroke(
                (float) len(0.028), BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
        g.draw(finger);
        g.setColor(FINGER);
        g.setStroke(
            new BasicStroke(
                (float) len(0.020), BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
        g.draw(finger);

        double pr = 0.012;
        g.setColor(ALUMINUM_LIGHT);
        g.fill(new Ellipse2D.Double(sx(pivotX - pr), sy(pivotY + pr), len(2 * pr), len(2 * pr)));
      }
    }

    private void drawSetpointMarkers(Graphics2D g, double rx, SimWorld.Snapshot s, double h) {
      g.setColor(SETPOINT);
      // Elevator setpoint marker
      double ey = s.elevatorSetpoint();
      double size = 0.03;
      triangle(g, sx(rx - 0.12), sy(ey), len(size), true);

      // Extender setpoint marker
      double ex = rx + WRIST_BASE_X + s.extenderSetpoint();
      triangle(g, sx(ex), sy(h + 0.2), len(size), false);
    }

    private void triangle(Graphics2D g, double px, double py, double size, boolean pointRight) {
      Path2D t = new Path2D.Double();
      t.moveTo(px, py);
      if (pointRight) {
        t.lineTo(px - size, py - size * 0.6);
        t.lineTo(px - size, py + size * 0.6);
      } else {
        t.lineTo(px - size * 0.6, py - size);
        t.lineTo(px + size * 0.6, py - size);
      }
      t.closePath();
      g.fill(t);
    }

    private void drawHud(Graphics2D g, SimWorld.Snapshot s, int w, int h) {
      int pad = 12;
      int lineH = 19;
      Font mono = new Font(Font.MONOSPACED, Font.PLAIN, 12);
      Font monoBold = new Font(Font.MONOSPACED, Font.BOLD, 12);
      Font bold = new Font(Font.SANS_SERIF, Font.BOLD, 14);

      String shooterPosLabel =
          String.format(
              "%.2f m %s",
              s.shooterPullback(),
              s.shooterSolenoidReleased() ? "[REL]" : (s.shooterLoaded() ? "[LOCK]" : ""));

      // Subsystem table columns: Subsystem | Current Position | Setpoint | Voltage | Status
      String[] headers = {"Subsystem", "Current Position", "Setpoint", "Voltage", "Status"};
      String[][] rows = {
        {"Drive", String.format("%.2f m", s.drivePosition()), String.format("%.2f m", s.driveSetpoint()), String.format("%5.1f V", s.driveVolts()), s.driveCommand()},
        {"Elevator", String.format("%.2f m", s.elevatorHeight()), String.format("%.2f m", s.elevatorSetpoint()), String.format("%5.1f V", s.elevatorVolts()), s.elevatorCommand()},
        {"Extender", String.format("%.2f m", s.extenderPosition()), String.format("%.2f m", s.extenderSetpoint()), String.format("%5.1f V", s.extenderVolts()), s.extenderCommand()},
        {"Claw", String.format("%.0f deg", Math.toDegrees(s.clawAngle())), String.format("%.0f deg", Math.toDegrees(s.clawSetpoint())), String.format("%5.1f V", s.clawVolts()), s.clawCommand()},
        {"Shooter", shooterPosLabel, String.format("%.2f m", s.shooterSetpoint()), String.format("%5.1f V", s.shooterVolts()), s.shooterCommand()},
      };

      int[] colWidths = {95, 135, 105, 85, 140};
      int totalTableW = 0;
      for (int wCol : colWidths) {
        totalTableW += wCol;
      }

      int boxW = totalTableW + 2 * pad;
      int boxH = pad * 2 + 18 + lineH * (rows.length + 2);
      int hudY = pad;
      g.setColor(HUD_BG);
      g.fillRoundRect(pad, hudY, boxW, boxH, 12, 12);

      // Mode & State Title
      g.setFont(bold);
      g.setColor(s.enabled() ? new Color(0x66FF88) : new Color(0xFF6666));
      g.drawString(
          s.mode().toUpperCase() + "  -  " + (s.enabled() ? "ENABLED" : "DISABLED"),
          2 * pad, hudY + pad + 12);

      // Header row
      int tableY = hudY + pad + 28;
      g.setFont(monoBold);
      g.setColor(new Color(0x93C5FD));
      int curX = 2 * pad;
      for (int c = 0; c < headers.length; c++) {
        g.drawString(headers[c], curX, tableY);
        curX += colWidths[c];
      }

      // Divider line under table header
      g.setColor(new Color(255, 255, 255, 45));
      g.setStroke(new BasicStroke(1f));
      g.drawLine(2 * pad, tableY + 5, 2 * pad + totalTableW - 10, tableY + 5);

      // Table data rows
      g.setFont(mono);
      SimWorld.MotorState[] motorStates = {
        s.driveMotorState(),
        s.elevatorMotorState(),
        s.extenderMotorState(),
        s.clawMotorState(),
        s.shooterMotorState()
      };
      for (int r = 0; r < rows.length; r++) {
        int rowY = tableY + 6 + lineH * (r + 1);
        curX = 2 * pad;
        SimWorld.MotorState mState = (r < motorStates.length) ? motorStates[r] : SimWorld.MotorState.IDLE;
        g.setColor(motorColor(mState));
        g.fillOval(curX, rowY - 8, 7, 7);
        for (int c = 0; c < rows[r].length; c++) {
          if (c == 0) {
            g.setColor(new Color(0xE2E8F0)); // Subsystem name
            g.drawString(rows[r][c], curX + 12, rowY);
          } else if (c == 4) {
            g.setColor(new Color(0x38BDF8)); // Status / command
            g.drawString(rows[r][c], curX, rowY);
          } else {
            g.setColor(Color.WHITE); // numeric metrics
            g.drawString(rows[r][c], curX, rowY);
          }
          curX += colWidths[c];
        }
      }

      // Status badges (top-right of elevation view)
      int scoreCount = (s.scored() ? 1 : 0) + (s.ball2Scored() ? 1 : 0);
      String scoreText = "Scored: " + scoreCount;
      g.setFont(new Font(Font.SANS_SERIF, Font.BOLD, 18));
      int tw = g.getFontMetrics().stringWidth(scoreText);
      g.setColor(HUD_BG);
      g.fillRoundRect(w - tw - 3 * pad, hudY, tw + 2 * pad, 36, 10, 10);
      g.setColor(scoreCount > 0 ? (scoreCount == 2 ? TARGET_GREEN_LIT : new Color(0x60A5FA)) : Color.WHITE);
      g.drawString(scoreText, w - tw - 2 * pad, hudY + 24);

      int nextBadgeY = hudY + 42;

      // Auto timer badge (displays during auto or after completion)
      if (s.autoRunning() || s.autoComplete() || s.autoTimeSeconds() > 0.0) {
        String timerText = String.format("Auto: %.2fs%s",
            s.autoTimeSeconds(),
            s.autoComplete() ? " (DONE)" : (s.autoRunning() ? "" : " (STOPPED)"));
        g.setFont(new Font(Font.SANS_SERIF, Font.BOLD, 14));
        int timerW = g.getFontMetrics().stringWidth(timerText);
        g.setColor(HUD_BG);
        g.fillRoundRect(w - timerW - 3 * pad, nextBadgeY, timerW + 2 * pad, 30, 8, 8);
        g.setColor(s.autoComplete() ? TARGET_GREEN_LIT : (s.autoRunning() ? new Color(0xFBBF24) : new Color(0x94A3B8)));
        g.drawString(timerText, w - timerW - 2 * pad, nextBadgeY + 20);
        nextBadgeY += 36;
      }

      // Ball held status badge
      if (s.ballHeld() || s.ball2Held()) {
        String heldText = (s.ballHeld() && s.ball2Held()) ? "Held: Both" : (s.ballHeld() ? "Held: Ball 1 (Grn)" : "Held: Ball 2 (Org)");
        g.setFont(new Font(Font.SANS_SERIF, Font.BOLD, 12));
        int hw = g.getFontMetrics().stringWidth(heldText);
        g.setColor(HUD_BG);
        g.fillRoundRect(w - hw - 3 * pad, nextBadgeY, hw + 2 * pad, 26, 8, 8);
        g.setColor(new Color(0xFFC04D));
        g.drawString(heldText, w - hw - 2 * pad, nextBadgeY + 18);
        nextBadgeY += 32;
      }

      if (s.tableContact()) {
        String contactText = "TABLE CONTACT!";
        g.setFont(new Font(Font.SANS_SERIF, Font.BOLD, 13));
        int cw = g.getFontMetrics().stringWidth(contactText);
        g.setColor(new Color(220, 38, 38, 220));
        g.fillRoundRect(w - cw - 3 * pad, nextBadgeY, cw + 2 * pad, 26, 8, 8);
        g.setColor(Color.WHITE);
        g.drawString(contactText, w - cw - 2 * pad, nextBadgeY + 18);
      }

      // Elevation View Title Badge
      g.setFont(new Font(Font.SANS_SERIF, Font.BOLD, 12));
      g.setColor(new Color(0x475569));
      g.drawString("SIDE ELEVATION VIEW (Robot Tracking)", pad + 4, hudY + boxH + 18);

      // Key legend (bottom-left)
      String[] help = {
        "D / A: drive forward / backward    Up / Down: elevator    Left / Right: extender",
        "Z / X: open / close claw    C / Space: pull back / kick shooter    T: Teleop    R: reset",
        isFocusOwnerWindow()
            ? "Keyboard active. Click buttons or use shortcuts T / R."
            : "Click this window to use the keyboard.",
      };
      g.setFont(new Font(Font.SANS_SERIF, Font.PLAIN, 12));
      int helpW = 0;
      for (String line : help) {
        helpW = Math.max(helpW, g.getFontMetrics().stringWidth(line));
      }
      int helpH = pad + lineH * help.length;
      g.setColor(HUD_BG);
      g.fillRoundRect(pad, h - helpH - pad, helpW + 2 * pad, helpH, 12, 12);
      for (int i = 0; i < help.length; i++) {
        g.setColor(i == 2 && !isFocusOwnerWindow() ? new Color(0xFF8A80) : Color.WHITE);
        g.drawString(help[i], 2 * pad, h - helpH - pad + lineH * (i + 1));
      }
    }

    private boolean isFocusOwnerWindow() {
      return SimWindow.this.isFocused();
    }

    private void label(Graphics2D g, String text, double x, double y) {
      g.setFont(new Font(Font.SANS_SERIF, Font.PLAIN, (int) Math.max(10, len(0.035))));
      g.setColor(new Color(0x333333));
      drawCentered(g, text, sx(x), sy(y));
    }

    private void drawCentered(Graphics2D g, String text, double cx, double baselineY) {
      int tw = g.getFontMetrics().stringWidth(text);
      g.drawString(text, (float) (cx - tw / 2.0), (float) baselineY);
    }
  }
}
