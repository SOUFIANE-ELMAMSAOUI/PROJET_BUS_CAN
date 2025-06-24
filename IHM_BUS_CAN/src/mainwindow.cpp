#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <iostream>
#include <QApplication>
#include <QHBoxLayout>
#include <cmath>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // Set window title
    this->setWindowTitle("MPU9250 Orientation Viewer");

    // Create and initialize the OpenGL object
    Object_GL = new ObjectOpenGL();
    Object_GL->setObjectName(QString::fromUtf8("ObjectOpenGL"));

    // Add the OpenGL widget to the UI
    QWidget* openglWidget = findChild<QWidget*>("openglWidget");
    if (openglWidget) {
        QHBoxLayout* layout = new QHBoxLayout(openglWidget);
        layout->setContentsMargins(0, 0, 0, 0);
        layout->addWidget(Object_GL);
        openglWidget->setLayout(layout);
    } else {
        std::cerr << "Error: Could not find openglWidget" << std::endl;
    }

    // Set up radio buttons
    ui->DistanceRadio->setChecked(true);

    // Create the View menu for the OpenGL widget
    QMenu *viewMenu = menuBar()->addMenu("&View");
    viewMenu->addAction("Front view", Object_GL, SLOT(FrontView()), QKeySequence(tr("Ctrl+f")));
    viewMenu->addAction("Rear view", Object_GL, SLOT(RearView()), QKeySequence(tr("Ctrl+e")));
    viewMenu->addAction("Left view", Object_GL, SLOT(LeftView()), QKeySequence(tr("Ctrl+l")));
    viewMenu->addAction("Right view", Object_GL, SLOT(RightView()), QKeySequence(tr("Ctrl+r")));
    viewMenu->addAction("Top view", Object_GL, SLOT(TopView()), QKeySequence(tr("Ctrl+t")));
    viewMenu->addAction("Bottom view", Object_GL, SLOT(BottomView()), QKeySequence(tr("Ctrl+b")));
    viewMenu->addSeparator();
    viewMenu->addAction("Isometric", Object_GL, SLOT(IsometricView()), QKeySequence(tr("Ctrl+i")));

    // Open CAN port
    openCANPort();

    // Create and start the timer for periodic CAN message polling
    timer_tick = new QTimer(this);
    connect(timer_tick, &QTimer::timeout, this, &MainWindow::onTimer_Tick);
    timer_tick->start(10);
}

MainWindow::~MainWindow()
{
    delete ui;
}

// Function to initialize and open the CAN socket
void MainWindow::openCANPort()
{
    if (socket_can.open("can0") == scpp::STATUS_OK)
    {
        std::cout << "CAN socket opened successfully" << std::endl;
    }
    else
    {
        std::cout << "Cannot open CAN socket!" << std::endl;
    }
}

// Handle sensor data from VL6180X (luminosity, distance, pressure, temperature)
void MainWindow::handleSensorData(const scpp::CanFrame& fr)
{
    switch(fr.id) {
    case 0x22: // Luminosity data
    {
        // First 4 bytes contain the luminosity value
        uint32_t luminosity = fr.data[0] |
                              (fr.data[1] << 8) |
                              (fr.data[2] << 16) |
                              (fr.data[3] << 24);
        ui->DistanceBrightnessValue->setText(QString::number(luminosity));
        ui->PressureLabel_3->setText("lux"); // Unit for luminosity
        ui->BrightnessRadio->setChecked(true);
        ui->DistanceRadio->setChecked(false);
    }
    break;

    case 0x23: // Distance data
    {
        // First 2 bytes contain the distance value
        uint16_t distance = fr.data[0] | (fr.data[1] << 8);
        ui->DistanceBrightnessValue->setText(QString::number(distance));
        ui->PressureLabel_3->setText("mm"); // Unit for distance
        ui->DistanceRadio->setChecked(true);
        ui->BrightnessRadio->setChecked(false);
    }
    break;

    case 0x24: // Pressure data
    {
        // 4 bytes contain the pressure value (scaled by 100)
        uint32_t pressure_raw = fr.data[0] |
                                (fr.data[1] << 8) |
                                (fr.data[2] << 16) |
                                (fr.data[3] << 24);
        float pressure_hpa = pressure_raw / 100.0f;
        ui->PressureLabel->setText(QString::number(pressure_hpa, 'f', 2));
    }
    break;

    case 0x25: // Temperature data
    {
        // 4 bytes contain the temperature value (scaled by 100)
        uint32_t temp_raw = fr.data[0] |
                            (fr.data[1] << 8) |
                            (fr.data[2] << 16) |
                            (fr.data[3] << 24);
        float temperature = temp_raw / 100.0f;
        ui->temperaturelabel->setText(QString::number(temperature, 'f', 2));
    }
    break;

    case 0x26: // Humidity data
    {
        // 4 bytes contain the humidity value (scaled by 100)
        uint32_t humid_raw = fr.data[0] |
                             (fr.data[1] << 8) |
                             (fr.data[2] << 16) |
                             (fr.data[3] << 24);
        float humidity = humid_raw / 100.0f;
        ui->HumidityLabel->setText(QString::number(humidity, 'f', 1));
    }
    break;
    }
}

// Handle data from the MPU9250 (orientation angles)
void MainWindow::handleAngleData(const scpp::CanFrame& fr)
{
    if (fr.id == 0x95) // ID for MPU9250 Euler angles
    {
        // Extract the Euler angles from the CAN frame (phi, theta, psi)
        int16_t phi = (fr.data[1] << 8) | fr.data[0];
        int16_t theta = (fr.data[3] << 8) | fr.data[2];
        int16_t psi = (fr.data[5] << 8) | fr.data[4];

        // Convert to degrees (values are sent as 100 times the actual value)
        float phi_deg = static_cast<float>(phi) / 100.0f;
        float theta_deg = static_cast<float>(theta) / 100.0f;
        float psi_deg = static_cast<float>(psi) / 100.0f;

        // Update UI labels with the angle values
        ui->phiLabel->setText(QString::number(phi_deg, 'f', 1) + "°");
        ui->ThetaLabel->setText(QString::number(theta_deg, 'f', 1) + "°");
        ui->PsiLabel->setText(QString::number(psi_deg, 'f', 1) + "°");

        // Update the OpenGL visualization with the new angles
        Object_GL->setAngles(phi_deg, theta_deg, psi_deg);
        // Convert to Qt's internal format (16-bit integers)
        Object_GL->SetXRotation((-phi_deg) * 16);
        Object_GL->SetYRotation((theta_deg) * 16);
        Object_GL->SetZRotation((-psi_deg) * 16);
        Object_GL->updateGL();
    }
}

// Handle wind speed data from the anemometer
void MainWindow::handleWindSpeedData(const scpp::CanFrame& fr)
{
    if (fr.id == 0x85) // ID for wind speed
    {
        // Extract wind speed data (2 bytes as per your code)
        uint16_t windSpeed = (fr.data[1] << 8) | fr.data[0];
        ui->windSpeedLabel->setText(QString::number(windSpeed));
    }
}
// Function to receive and process CAN messages
void MainWindow::receiveCANMessage()
{
    scpp::CanFrame fr;

    if (socket_can.read(fr) == scpp::STATUS_OK)
    {
        // Process the message based on its ID
        if (fr.id >= 0x22 && fr.id <= 0x26) {
            // VL6180X and environmental sensor data
            handleSensorData(fr);
        }
        else if (fr.id == 0x85) {
            // Wind speed data
            handleWindSpeedData(fr);
        }
        else if (fr.id == 0x95) {
            // MPU9250 angle data
            handleAngleData(fr);
        }

        // Update the UI
        update();
    }
}

// Timer callback function - called periodically to check for new CAN messages
void MainWindow::onTimer_Tick()
{
    receiveCANMessage();
    Object_GL->updateGL();
}

// Send motor speed command when button is clicked
void MainWindow::on_setSpeed_clicked()
{
    bool ok;
    int speed = ui->MotorSpeed->text().toInt(&ok);

    if (ok && speed >= 0 && speed <= 255) {
        scpp::CanFrame frame;
        frame.id = 0x03;
        frame.len = 2;
        frame.data[0] = 0x40;
        frame.data[1] = static_cast<uint8_t>(speed);

        if (socket_can.write(frame) == scpp::STATUS_OK) {
            std::cout << "Sent motor speed command: " << speed << std::endl;
        } else {
            std::cerr << "Failed to send motor speed command" << std::endl;
        }
    } else {
        std::cerr << "Invalid speed value: " << ui->MotorSpeed->text().toStdString() << std::endl;
    }
}
// Send a command to switch to distance mode
void MainWindow::on_DistanceRadio_clicked(bool checked)
{
    if (checked) {
        ui->BrightnessRadio->setChecked(false);
        sendDistanceModeCommand();
    }
}

void MainWindow::on_BrightnessRadio_clicked(bool checked)
{
    if (checked) {
        ui->DistanceRadio->setChecked(false);
        sendBrightnessModeCommand();
    }
}

// Function to send motor speed command via CAN
void MainWindow::sendMotorCommand(uint8_t speed)
{
    scpp::CanFrame frame;
    frame.id = 0x40;
    frame.len = 2;
    frame.data[0] = 0x40;
    frame.data[1] = speed;

    socket_can.write(frame);
}

// Function to send command to switch to distance mode
void MainWindow::sendDistanceModeCommand()
{
    scpp::CanFrame frame;
    frame.id = 0x01;
    frame.len = 8;

    // Set all data bytes to 0 except the last one
    for (int i = 0; i < 7; i++) {
        frame.data[i] = 0;
    }
    frame.data[7] = 0x12; // Command for distance mode

    socket_can.write(frame);
    std::cout << "Sent distance mode command" << std::endl;
}

// Function to send command to switch to luminosity mode
void MainWindow::sendBrightnessModeCommand()
{
    scpp::CanFrame frame;
    frame.id = 0x01;
    frame.len = 8;

    // Set all data bytes to 0 except the last one
    for (int i = 0; i < 7; i++) {
        frame.data[i] = 0;
    }
    frame.data[7] = 0x11;

    socket_can.write(frame);
    std::cout << "Sent luminosity mode command" << std::endl;
}
