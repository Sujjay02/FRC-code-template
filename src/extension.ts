import * as vscode from 'vscode';
import * as fs from 'fs';
import * as path from 'path';

import { getProjectInfo, langDisplay } from './projectInfo';

import {
    tplSwerveModuleRev, tplSwerveModuleCtre,
    tplSwerveDriveNavx, tplSwerveDrivePigeon,
    tplTankRev, tplTankCtre, tplMecanum,
} from './templates/drivetrains';

import {
    tplShooter, tplHopper, tplTurret, tplPuncher, tplCatapult,
} from './templates/shooting';

import {
    tplIntake, tplDeployIntake, tplConveyor, tplSerializer,
} from './templates/handling';

import {
    tplElevatorRev, tplElevatorMM,
    tplArmRev, tplArmMM,
    tplDoubleArm, tplWrist, tplTelescope, tplMechanism,
} from './templates/arms';

import {
    tplClimb, tplBuddyClimb, tplClaw, tplPneumatics,
} from './templates/endgame';

import {
    tplVisionPhoton, tplVisionLimelight, tplLeds,
} from './templates/sensors';

import { tplPathPlanner, tplChoreo } from './templates/autonomous';

// ─── helpers ─────────────────────────────────────────────────────────────────

function findRobotDir(wsRoot: string): string | undefined {
    const hasBuild = fs.existsSync(path.join(wsRoot, 'build.gradle'));
    const hasSrc   = fs.existsSync(path.join(wsRoot, 'src', 'main', 'java', 'frc', 'robot'));
    if (!hasBuild && !hasSrc) { return undefined; }
    return path.join(wsRoot, 'src', 'main', 'java', 'frc', 'robot');
}

async function requireRobotDir(): Promise<string | undefined> {
    const folders = vscode.workspace.workspaceFolders;
    if (!folders?.length) {
        vscode.window.showErrorMessage('Open a WPILib project folder first.');
        return undefined;
    }
    const dir = findRobotDir(folders[0].uri.fsPath);
    if (!dir) {
        vscode.window.showErrorMessage('No WPILib project detected (missing build.gradle / src/main/java/frc/robot).');
        return undefined;
    }
    return dir;
}

async function writeSafe(filePath: string, content: string): Promise<boolean> {
    if (fs.existsSync(filePath)) {
        const ans = await vscode.window.showWarningMessage(
            `${path.basename(filePath)} already exists — overwrite?`, { modal: true }, 'Overwrite');
        if (ans !== 'Overwrite') { return false; }
    }
    fs.mkdirSync(path.dirname(filePath), { recursive: true });
    fs.writeFileSync(filePath, content, 'utf8');
    return true;
}

async function reveal(filePath: string) {
    const doc = await vscode.workspace.openTextDocument(vscode.Uri.file(filePath));
    vscode.window.showTextDocument(doc, { preview: false });
}

function qp(items: string[], placeholder: string) {
    return vscode.window.showQuickPick(items, { placeHolder: placeholder });
}

// ─── command handlers ────────────────────────────────────────────────────────

async function cmdSwerve() {
    const robotDir = await requireRobotDir(); if (!robotDir) { return; }
    const motor = await qp(['REV (MAXSwerve / SparkMax)', 'CTRE (TalonFX + CANcoder)'], 'Swerve module hardware');
    if (!motor) { return; }
    const gyro = await qp(['NavX (Studica)', 'Pigeon 2 (CTRE)'], 'Gyro');
    if (!gyro) { return; }
    const isRev = motor.startsWith('REV'), isPigeon = gyro.startsWith('Pigeon');
    const subDir = path.join(robotDir, 'subsystems');
    const modOk  = await writeSafe(path.join(subDir, 'SwerveModule.java'),
        isRev ? tplSwerveModuleRev() : tplSwerveModuleCtre());
    const drv    = path.join(subDir, 'SwerveDriveSubsystem.java');
    const drvOk  = await writeSafe(drv, isPigeon ? tplSwerveDrivePigeon(isRev) : tplSwerveDriveNavx(isRev));
    if (drvOk) { await reveal(drv); } else if (modOk) { await reveal(path.join(subDir, 'SwerveModule.java')); }
    vscode.window.showInformationMessage(`FRC Jumpstart: Swerve written (${isRev ? 'REV' : 'CTRE'}, ${isPigeon ? 'Pigeon 2' : 'NavX'}). Fill in DriveConstants + ModuleConstants.`);
}

async function cmdTank() {
    const robotDir = await requireRobotDir(); if (!robotDir) { return; }
    const motor = await qp(['REV (SparkMax)', 'CTRE (TalonFX)'], 'Motor hardware');
    if (!motor) { return; }
    const isRev = motor.startsWith('REV');
    const p = path.join(robotDir, 'subsystems', 'TankDriveSubsystem.java');
    const ok = await writeSafe(p, isRev ? tplTankRev() : tplTankCtre());
    if (ok) { await reveal(p); }
    vscode.window.showInformationMessage('FRC Jumpstart: TankDriveSubsystem.java written. Fill in DriveConstants.');
}

async function cmdMecanum() {
    const robotDir = await requireRobotDir(); if (!robotDir) { return; }
    const p = path.join(robotDir, 'subsystems', 'MecanumDriveSubsystem.java');
    const ok = await writeSafe(p, tplMecanum());
    if (ok) { await reveal(p); }
    vscode.window.showInformationMessage('FRC Jumpstart: MecanumDriveSubsystem.java written. Fill in DriveConstants CAN IDs.');
}

async function cmdShooter() {
    const robotDir = await requireRobotDir(); if (!robotDir) { return; }
    const p = path.join(robotDir, 'subsystems', 'ShooterSubsystem.java');
    const ok = await writeSafe(p, tplShooter());
    if (ok) { await reveal(p); }
    vscode.window.showInformationMessage('FRC Jumpstart: ShooterSubsystem.java written. Set CAN IDs and tune PID.');
}

async function cmdShooterHopper() {
    const robotDir = await requireRobotDir(); if (!robotDir) { return; }
    const sp = path.join(robotDir, 'subsystems', 'ShooterSubsystem.java');
    const hp = path.join(robotDir, 'subsystems', 'HopperSubsystem.java');
    const s = await writeSafe(sp, tplShooter());
    const h = await writeSafe(hp, tplHopper());
    if (h) { await reveal(hp); } else if (s) { await reveal(sp); }
    vscode.window.showInformationMessage('FRC Jumpstart: ShooterSubsystem + HopperSubsystem written. Set CAN IDs and DIO ports.');
}

async function cmdTurret() {
    const robotDir = await requireRobotDir(); if (!robotDir) { return; }
    const p = path.join(robotDir, 'subsystems', 'TurretSubsystem.java');
    const ok = await writeSafe(p, tplTurret());
    if (ok) { await reveal(p); }
    vscode.window.showInformationMessage('FRC Jumpstart: TurretSubsystem.java written. Set CAN ID and tune angle presets.');
}

async function cmdPuncher() {
    const robotDir = await requireRobotDir(); if (!robotDir) { return; }
    const p = path.join(robotDir, 'subsystems', 'PuncherSubsystem.java');
    const ok = await writeSafe(p, tplPuncher());
    if (ok) { await reveal(p); }
    vscode.window.showInformationMessage('FRC Jumpstart: PuncherSubsystem.java written. Set solenoid channels.');
}

async function cmdCatapult() {
    const robotDir = await requireRobotDir(); if (!robotDir) { return; }
    const p = path.join(robotDir, 'subsystems', 'CatapultSubsystem.java');
    const ok = await writeSafe(p, tplCatapult());
    if (ok) { await reveal(p); }
    vscode.window.showInformationMessage('FRC Jumpstart: CatapultSubsystem.java written. Set CAN ID and home sensor DIO port.');
}

async function cmdIntake() {
    const robotDir = await requireRobotDir(); if (!robotDir) { return; }
    const p = path.join(robotDir, 'subsystems', 'IntakeSubsystem.java');
    const ok = await writeSafe(p, tplIntake());
    if (ok) { await reveal(p); }
    vscode.window.showInformationMessage('FRC Jumpstart: IntakeSubsystem.java written. Set CAN ID and DIO port.');
}

async function cmdDeployIntake() {
    const robotDir = await requireRobotDir(); if (!robotDir) { return; }
    const p = path.join(robotDir, 'subsystems', 'IntakeSubsystem.java');
    const ok = await writeSafe(p, tplDeployIntake());
    if (ok) { await reveal(p); }
    vscode.window.showInformationMessage('FRC Jumpstart: IntakeSubsystem.java (deploy arm + roller) written. Set CAN IDs and tune arm positions.');
}

async function cmdConveyor() {
    const robotDir = await requireRobotDir(); if (!robotDir) { return; }
    const p = path.join(robotDir, 'subsystems', 'ConveyorSubsystem.java');
    const ok = await writeSafe(p, tplConveyor());
    if (ok) { await reveal(p); }
    vscode.window.showInformationMessage('FRC Jumpstart: ConveyorSubsystem.java written. Set CAN ID and DIO port.');
}

async function cmdSerializer() {
    const robotDir = await requireRobotDir(); if (!robotDir) { return; }
    const p = path.join(robotDir, 'subsystems', 'SerializerSubsystem.java');
    const ok = await writeSafe(p, tplSerializer());
    if (ok) { await reveal(p); }
    vscode.window.showInformationMessage('FRC Jumpstart: SerializerSubsystem.java written. Set CAN IDs and DIO ports.');
}

async function cmdElevator() {
    const robotDir = await requireRobotDir(); if (!robotDir) { return; }
    const motor = await qp(['REV (SparkMax)', 'CTRE (MotionMagic / TalonFX)'], 'Motor hardware');
    if (!motor) { return; }
    const isMM = motor.startsWith('CTRE');
    const p = path.join(robotDir, 'subsystems', 'ElevatorSubsystem.java');
    const ok = await writeSafe(p, isMM ? tplElevatorMM() : tplElevatorRev());
    if (ok) { await reveal(p); }
    vscode.window.showInformationMessage(`FRC Jumpstart: ElevatorSubsystem.java written (${isMM ? 'CTRE MotionMagic' : 'REV SparkMax'}). Set CAN IDs and tune heights.`);
}

async function cmdArm() {
    const robotDir = await requireRobotDir(); if (!robotDir) { return; }
    const motor = await qp(['REV (SparkMax + AbsoluteEncoder)', 'CTRE (MotionMagic / TalonFX)'], 'Motor hardware');
    if (!motor) { return; }
    const isMM = motor.startsWith('CTRE');
    const p = path.join(robotDir, 'subsystems', 'ArmSubsystem.java');
    const ok = await writeSafe(p, isMM ? tplArmMM() : tplArmRev());
    if (ok) { await reveal(p); }
    vscode.window.showInformationMessage(`FRC Jumpstart: ArmSubsystem.java written (${isMM ? 'CTRE MotionMagic' : 'REV SparkMax'}). Set CAN ID and tune angle presets.`);
}

async function cmdDoubleArm() {
    const robotDir = await requireRobotDir(); if (!robotDir) { return; }
    const p = path.join(robotDir, 'subsystems', 'ArmSubsystem.java');
    const ok = await writeSafe(p, tplDoubleArm());
    if (ok) { await reveal(p); }
    vscode.window.showInformationMessage('FRC Jumpstart: ArmSubsystem.java (double-jointed) written. Set CAN IDs and tune shoulder + elbow presets.');
}

async function cmdWrist() {
    const robotDir = await requireRobotDir(); if (!robotDir) { return; }
    const p = path.join(robotDir, 'subsystems', 'WristSubsystem.java');
    const ok = await writeSafe(p, tplWrist());
    if (ok) { await reveal(p); }
    vscode.window.showInformationMessage('FRC Jumpstart: WristSubsystem.java written. Set CAN ID and tune angle presets.');
}

async function cmdTelescope() {
    const robotDir = await requireRobotDir(); if (!robotDir) { return; }
    const p = path.join(robotDir, 'subsystems', 'TelescopeSubsystem.java');
    const ok = await writeSafe(p, tplTelescope());
    if (ok) { await reveal(p); }
    vscode.window.showInformationMessage('FRC Jumpstart: TelescopeSubsystem.java written. Set CAN ID and tune extension positions.');
}

async function cmdMechanism() {
    const robotDir = await requireRobotDir(); if (!robotDir) { return; }
    const name = await vscode.window.showInputBox({
        prompt: 'Subsystem name (PascalCase, e.g. Arm, Wrist, Indexer)',
        validateInput: v => /^[A-Z][A-Za-z0-9]*$/.test(v) ? null : 'Must start with capital letter, letters/digits only',
    });
    if (!name) { return; }
    const presetInput = await vscode.window.showInputBox({
        prompt: 'Preset positions (comma-separated)',
        value: 'stow,intake,score',
    });
    if (presetInput === undefined) { return; }
    const presets = presetInput.split(',').map(s => s.trim()).filter(Boolean);
    const p = path.join(robotDir, 'subsystems', `${name}Subsystem.java`);
    const ok = await writeSafe(p, tplMechanism(name, presets));
    if (ok) { await reveal(p); }
    vscode.window.showInformationMessage(`FRC Jumpstart: ${name}Subsystem.java written with presets: ${presets.join(', ')}.`);
}

async function cmdClimb() {
    const robotDir = await requireRobotDir(); if (!robotDir) { return; }
    const p = path.join(robotDir, 'subsystems', 'ClimbSubsystem.java');
    const ok = await writeSafe(p, tplClimb());
    if (ok) { await reveal(p); }
    vscode.window.showInformationMessage('FRC Jumpstart: ClimbSubsystem.java written. Set CAN IDs and tune extend position.');
}

async function cmdBuddyClimb() {
    const robotDir = await requireRobotDir(); if (!robotDir) { return; }
    const p = path.join(robotDir, 'subsystems', 'BuddyClimbSubsystem.java');
    const ok = await writeSafe(p, tplBuddyClimb());
    if (ok) { await reveal(p); }
    vscode.window.showInformationMessage('FRC Jumpstart: BuddyClimbSubsystem.java written. Set CAN ID and tune deploy position.');
}

async function cmdClaw() {
    const robotDir = await requireRobotDir(); if (!robotDir) { return; }
    const p = path.join(robotDir, 'subsystems', 'ClawSubsystem.java');
    const ok = await writeSafe(p, tplClaw());
    if (ok) { await reveal(p); }
    vscode.window.showInformationMessage('FRC Jumpstart: ClawSubsystem.java written. Set CAN ID and DIO port.');
}

async function cmdPneumatics() {
    const robotDir = await requireRobotDir(); if (!robotDir) { return; }
    const p = path.join(robotDir, 'subsystems', 'PneumaticsSubsystem.java');
    const ok = await writeSafe(p, tplPneumatics());
    if (ok) { await reveal(p); }
    vscode.window.showInformationMessage('FRC Jumpstart: PneumaticsSubsystem.java written. Set solenoid channels and module type.');
}

async function cmdVision() {
    const robotDir = await requireRobotDir(); if (!robotDir) { return; }
    const lib = await qp(['PhotonVision (PhotonPoseEstimator)', 'Limelight (NetworkTables 4)'], 'Vision library');
    if (!lib) { return; }
    const isPhoton = lib.startsWith('Photo');
    const p = path.join(robotDir, 'subsystems', 'VisionSubsystem.java');
    const ok = await writeSafe(p, isPhoton ? tplVisionPhoton() : tplVisionLimelight());
    if (ok) { await reveal(p); }
    vscode.window.showInformationMessage(`FRC Jumpstart: VisionSubsystem.java written (${isPhoton ? 'PhotonVision' : 'Limelight'}).`);
}

async function cmdLeds() {
    const robotDir = await requireRobotDir(); if (!robotDir) { return; }
    const p = path.join(robotDir, 'subsystems', 'LEDSubsystem.java');
    const ok = await writeSafe(p, tplLeds());
    if (ok) { await reveal(p); }
    vscode.window.showInformationMessage('FRC Jumpstart: LEDSubsystem.java written. Set kPort and kLength.');
}

async function cmdPathPlanner() {
    const robotDir = await requireRobotDir(); if (!robotDir) { return; }
    const p = path.join(robotDir, 'AutoConfig.java');
    const ok = await writeSafe(p, tplPathPlanner());
    if (ok) { await reveal(p); }
    vscode.window.showInformationMessage('FRC Jumpstart: AutoConfig.java written. Instantiate in RobotContainer and register NamedCommands.');
}

async function cmdChoreo() {
    const robotDir = await requireRobotDir(); if (!robotDir) { return; }
    const p = path.join(robotDir, 'ChoreoAutoConfig.java');
    const ok = await writeSafe(p, tplChoreo());
    if (ok) { await reveal(p); }
    vscode.window.showInformationMessage('FRC Jumpstart: ChoreoAutoConfig.java written. Add choreolib dependency and place .traj files in deploy/choreo/.');
}

async function cmdCombine() {
    const robotDir = await requireRobotDir(); if (!robotDir) { return; }

    interface Item extends vscode.QuickPickItem { id: string; }
    const items: Item[] = [
        { label: 'Swerve Drive',           id: 'swerve',        description: 'SwerveDriveSubsystem + SwerveModule' },
        { label: 'Tank Drive',             id: 'tank',           description: 'TankDriveSubsystem' },
        { label: 'Mecanum Drive',          id: 'mecanum',        description: 'MecanumDriveSubsystem' },
        { label: 'Flywheel Shooter',       id: 'shooter',        description: 'ShooterSubsystem' },
        { label: 'Shooter + Hopper',       id: 'shooterhopper',  description: 'ShooterSubsystem + HopperSubsystem' },
        { label: 'Turret',                 id: 'turret',         description: 'TurretSubsystem' },
        { label: 'Puncher / Kicker',       id: 'puncher',        description: 'PuncherSubsystem (solenoid)' },
        { label: 'Catapult',               id: 'catapult',       description: 'CatapultSubsystem' },
        { label: 'Intake (Roller)',        id: 'intake',         description: 'IntakeSubsystem' },
        { label: 'Intake with Deploy Arm', id: 'deployintake',   description: 'IntakeSubsystem (pivot + roller)' },
        { label: 'Hopper / Indexer',       id: 'hopper',         description: 'HopperSubsystem' },
        { label: 'Conveyor / Belt',        id: 'conveyor',       description: 'ConveyorSubsystem' },
        { label: 'Serializer',             id: 'serializer',     description: 'SerializerSubsystem (two-zone)' },
        { label: 'Elevator',               id: 'elevator',       description: 'ElevatorSubsystem (REV or MotionMagic)' },
        { label: 'Pivot Arm',              id: 'arm',            description: 'ArmSubsystem (REV or MotionMagic)' },
        { label: 'Double-Jointed Arm',     id: 'doublearm',      description: 'ArmSubsystem (shoulder + elbow)' },
        { label: 'Wrist',                  id: 'wrist',          description: 'WristSubsystem' },
        { label: 'Telescope / Extension',  id: 'telescope',      description: 'TelescopeSubsystem' },
        { label: 'Generic Mechanism',      id: 'mechanism',      description: 'Custom name + preset positions' },
        { label: 'Climb',                  id: 'climb',          description: 'ClimbSubsystem' },
        { label: 'Buddy Climb',            id: 'buddyclimb',     description: 'BuddyClimbSubsystem' },
        { label: 'Claw / Gripper',         id: 'claw',           description: 'ClawSubsystem' },
        { label: 'Pneumatics',             id: 'pneumatics',     description: 'PneumaticsSubsystem' },
        { label: 'Vision',                 id: 'vision',         description: 'VisionSubsystem (PhotonVision or Limelight)' },
        { label: 'Addressable LEDs',       id: 'leds',           description: 'LEDSubsystem' },
        { label: 'PathPlanner Autos',      id: 'pathplanner',    description: 'AutoConfig.java' },
        { label: 'Choreo Autos',           id: 'choreo',         description: 'ChoreoAutoConfig.java' },
    ];

    const selected = await vscode.window.showQuickPick(items, {
        canPickMany: true,
        placeHolder: 'Space to select, Enter to generate',
        title: 'FRC Jumpstart — Combine Templates',
    }) as Item[] | undefined;
    if (!selected?.length) { return; }

    for (const item of selected) {
        switch (item.id) {
            case 'swerve':       await cmdSwerve();       break;
            case 'tank':         await cmdTank();         break;
            case 'mecanum':      await cmdMecanum();      break;
            case 'shooter':      await cmdShooter();      break;
            case 'shooterhopper':await cmdShooterHopper();break;
            case 'turret':       await cmdTurret();       break;
            case 'puncher':      await cmdPuncher();      break;
            case 'catapult':     await cmdCatapult();     break;
            case 'intake':       await cmdIntake();       break;
            case 'deployintake': await cmdDeployIntake(); break;
            case 'hopper':       await cmdShooterHopper();break;
            case 'conveyor':     await cmdConveyor();     break;
            case 'serializer':   await cmdSerializer();   break;
            case 'elevator':     await cmdElevator();     break;
            case 'arm':          await cmdArm();          break;
            case 'doublearm':    await cmdDoubleArm();    break;
            case 'wrist':        await cmdWrist();        break;
            case 'telescope':    await cmdTelescope();    break;
            case 'mechanism':    await cmdMechanism();    break;
            case 'climb':        await cmdClimb();        break;
            case 'buddyclimb':   await cmdBuddyClimb();  break;
            case 'claw':         await cmdClaw();         break;
            case 'pneumatics':   await cmdPneumatics();   break;
            case 'vision':       await cmdVision();       break;
            case 'leds':         await cmdLeds();         break;
            case 'pathplanner':  await cmdPathPlanner();  break;
            case 'choreo':       await cmdChoreo();       break;
        }
    }
}

async function cmdOpen() {
    const choice = await qp([
        'Swerve Drive', 'Tank Drive', 'Mecanum Drive',
        'Flywheel Shooter', 'Shooter + Hopper', 'Turret', 'Puncher / Kicker', 'Catapult',
        'Intake (Roller)', 'Intake with Deploy Arm', 'Hopper / Indexer', 'Conveyor / Belt', 'Serializer',
        'Elevator', 'Pivot Arm', 'Double-Jointed Arm', 'Wrist', 'Telescope / Extension', 'Generic Mechanism',
        'Climb', 'Buddy Climb', 'Claw / Gripper', 'Pneumatics',
        'Vision', 'Addressable LEDs',
        'PathPlanner Autos', 'Choreo Autos',
        'Combine Templates',
    ], 'FRC Jumpstart — choose a template');
    if (!choice) { return; }
    const map: Record<string, () => Promise<void>> = {
        'Swerve Drive': cmdSwerve, 'Tank Drive': cmdTank, 'Mecanum Drive': cmdMecanum,
        'Flywheel Shooter': cmdShooter, 'Shooter + Hopper': cmdShooterHopper,
        'Turret': cmdTurret, 'Puncher / Kicker': cmdPuncher, 'Catapult': cmdCatapult,
        'Intake (Roller)': cmdIntake, 'Intake with Deploy Arm': cmdDeployIntake,
        'Hopper / Indexer': cmdShooterHopper, 'Conveyor / Belt': cmdConveyor, 'Serializer': cmdSerializer,
        'Elevator': cmdElevator, 'Pivot Arm': cmdArm, 'Double-Jointed Arm': cmdDoubleArm,
        'Wrist': cmdWrist, 'Telescope / Extension': cmdTelescope, 'Generic Mechanism': cmdMechanism,
        'Climb': cmdClimb, 'Buddy Climb': cmdBuddyClimb, 'Claw / Gripper': cmdClaw, 'Pneumatics': cmdPneumatics,
        'Vision': cmdVision, 'Addressable LEDs': cmdLeds,
        'PathPlanner Autos': cmdPathPlanner, 'Choreo Autos': cmdChoreo,
        'Combine Templates': cmdCombine,
    };
    await map[choice]?.();
}

// ─── side panel ──────────────────────────────────────────────────────────────

class FrcForgeViewProvider implements vscode.WebviewViewProvider {
    static readonly viewId = 'frcJumpstart.panel';

    resolveWebviewView(view: vscode.WebviewView) {
        view.webview.options = { enableScripts: true };

        const refresh = () => { view.webview.html = this._html(getProjectInfo()); };
        refresh();

        const watcher = vscode.workspace.onDidChangeWorkspaceFolders(refresh);
        view.onDidDispose(() => watcher.dispose());

        view.webview.onDidReceiveMessage((msg: { command: string }) => {
            const map: Record<string, () => void> = {
                swerve: cmdSwerve, tank: cmdTank, mecanum: cmdMecanum,
                shooter: cmdShooter, shooterhopper: cmdShooterHopper,
                turret: cmdTurret, puncher: cmdPuncher, catapult: cmdCatapult,
                intake: cmdIntake, deployintake: cmdDeployIntake,
                hopper: () => { const h = path.join('', 'HopperSubsystem.java'); void h; cmdShooterHopper(); },
                conveyor: cmdConveyor, serializer: cmdSerializer,
                elevator: cmdElevator, arm: cmdArm, doublearm: cmdDoubleArm,
                wrist: cmdWrist, telescope: cmdTelescope, mechanism: cmdMechanism,
                climb: cmdClimb, buddyclimb: cmdBuddyClimb,
                claw: cmdClaw, pneumatics: cmdPneumatics,
                vision: cmdVision, leds: cmdLeds,
                pathplanner: cmdPathPlanner, choreo: cmdChoreo,
                combine: cmdCombine,
            };
            map[msg.command]?.();
        });
    }

    private _html(info: ReturnType<typeof getProjectInfo>): string {
        const nonce = Math.random().toString(36).substring(2, 15);

        const statusText = info
            ? `WPILib ${info.year}  ·  ${langDisplay(info.language)}`
            : 'No WPILib project detected';

        const row = (id: string, label: string) =>
            `<button class="row" data-cmd="${id}">${label}</button>`;

        const section = (title: string, rows: string) =>
            `<div class="pane"><div class="pane-header">${title}</div>${rows}</div>`;

        return `<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta http-equiv="Content-Security-Policy" content="default-src 'none'; style-src 'unsafe-inline'; script-src 'nonce-${nonce}';">
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; }
  html, body { height: 100%; overflow: hidden; }
  body {
    display: flex; flex-direction: column;
    font-family: var(--vscode-font-family);
    font-size: var(--vscode-font-size, 13px);
    color: var(--vscode-foreground);
    background: var(--vscode-sideBar-background);
  }
  .scroll { flex: 1; overflow-y: auto; overflow-x: hidden; }
  .pane { border-top: 1px solid var(--vscode-sideBarSectionHeader-border, rgba(128,128,128,.35)); }
  .pane-header {
    display: flex; align-items: center; height: 22px; padding: 0 12px;
    font-size: 11px; font-weight: 700; letter-spacing: .08em; text-transform: uppercase;
    color: var(--vscode-sideBarSectionHeader-foreground, var(--vscode-foreground));
    background: var(--vscode-sideBarSectionHeader-background, transparent);
    user-select: none;
  }
  .row {
    display: block; width: 100%; height: 22px; line-height: 22px;
    padding: 0 0 0 28px; background: transparent;
    color: var(--vscode-foreground); border: none; cursor: pointer;
    font-size: var(--vscode-font-size, 13px); font-family: var(--vscode-font-family);
    text-align: left; white-space: nowrap; overflow: hidden; text-overflow: ellipsis;
  }
  .row:hover { background: var(--vscode-list-hoverBackground); }
  .row:focus-visible { outline: 1px solid var(--vscode-focusBorder); outline-offset: -1px; }
  .combine-wrap {
    padding: 8px 12px 10px;
    border-top: 1px solid var(--vscode-sideBarSectionHeader-border, rgba(128,128,128,.35));
  }
  .combine-btn {
    width: 100%; padding: 4px 0; border: none; border-radius: 2px; cursor: pointer;
    background: var(--vscode-button-background, #0e639c);
    color: var(--vscode-button-foreground, #fff);
    font-size: var(--vscode-font-size, 13px); font-family: var(--vscode-font-family);
  }
  .combine-btn:hover { background: var(--vscode-button-hoverBackground, #1177bb); }
  .combine-btn:focus-visible { outline: 1px solid var(--vscode-focusBorder); outline-offset: 2px; }
  .status-bar {
    flex-shrink: 0; display: flex; align-items: center; height: 22px; padding: 0 12px;
    font-size: 11px; user-select: none;
    color: var(--vscode-descriptionForeground, rgba(204,204,204,.7));
    background: var(--vscode-sideBarSectionHeader-background, transparent);
    border-top: 1px solid var(--vscode-sideBarSectionHeader-border, rgba(128,128,128,.35));
  }
</style>
</head>
<body>
<div class="scroll">

${section('Drivetrains',
    row('swerve',  'Swerve Drive') +
    row('tank',    'Tank Drive') +
    row('mecanum', 'Mecanum Drive')
)}

${section('Shooting',
    row('shooter',      'Flywheel Shooter') +
    row('shooterhopper','Shooter + Hopper') +
    row('turret',       'Turret') +
    row('puncher',      'Puncher / Kicker') +
    row('catapult',     'Catapult')
)}

${section('Game Piece Handling',
    row('intake',       'Intake (Roller)') +
    row('deployintake', 'Intake with Deploy Arm') +
    row('conveyor',     'Conveyor / Belt') +
    row('serializer',   'Serializer')
)}

${section('Arms &amp; Elevators',
    row('elevator',  'Elevator') +
    row('arm',       'Pivot Arm') +
    row('doublearm', 'Double-Jointed Arm') +
    row('wrist',     'Wrist') +
    row('telescope', 'Telescope / Extension') +
    row('mechanism', 'Generic Mechanism')
)}

${section('Endgame',
    row('climb',      'Climb') +
    row('buddyclimb', 'Buddy Climb') +
    row('claw',       'Claw / Gripper') +
    row('pneumatics', 'Pneumatics')
)}

${section('Sensors &amp; Visualization',
    row('vision', 'Vision') +
    row('leds',   'Addressable LEDs')
)}

${section('Autonomous',
    row('pathplanner', 'PathPlanner Autos') +
    row('choreo',      'Choreo Autos')
)}

<div class="combine-wrap">
  <button class="combine-btn" data-cmd="combine">Combine Templates</button>
</div>

</div><!-- end .scroll -->

<div class="status-bar">${statusText}</div>

<script nonce="${nonce}">
  const vscode = acquireVsCodeApi();
  document.querySelectorAll('[data-cmd]').forEach(el =>
    el.addEventListener('click', () => vscode.postMessage({ command: el.dataset.cmd }))
  );
</script>
</body>
</html>`;
    }
}

// ─── activate ────────────────────────────────────────────────────────────────

export function activate(context: vscode.ExtensionContext) {
    context.subscriptions.push(
        vscode.commands.registerCommand('frcJumpstart.open',             cmdOpen),
        vscode.commands.registerCommand('frcJumpstart.newSwerve',        cmdSwerve),
        vscode.commands.registerCommand('frcJumpstart.newTank',          cmdTank),
        vscode.commands.registerCommand('frcJumpstart.newMecanum',       cmdMecanum),
        vscode.commands.registerCommand('frcJumpstart.newShooter',       cmdShooter),
        vscode.commands.registerCommand('frcJumpstart.newShooterHopper', cmdShooterHopper),
        vscode.commands.registerCommand('frcJumpstart.newTurret',        cmdTurret),
        vscode.commands.registerCommand('frcJumpstart.newPuncher',       cmdPuncher),
        vscode.commands.registerCommand('frcJumpstart.newCatapult',      cmdCatapult),
        vscode.commands.registerCommand('frcJumpstart.newIntake',        cmdIntake),
        vscode.commands.registerCommand('frcJumpstart.newDeployIntake',  cmdDeployIntake),
        vscode.commands.registerCommand('frcJumpstart.newConveyor',      cmdConveyor),
        vscode.commands.registerCommand('frcJumpstart.newSerializer',    cmdSerializer),
        vscode.commands.registerCommand('frcJumpstart.newElevator',      cmdElevator),
        vscode.commands.registerCommand('frcJumpstart.newArm',           cmdArm),
        vscode.commands.registerCommand('frcJumpstart.newDoubleArm',     cmdDoubleArm),
        vscode.commands.registerCommand('frcJumpstart.newWrist',         cmdWrist),
        vscode.commands.registerCommand('frcJumpstart.newTelescope',     cmdTelescope),
        vscode.commands.registerCommand('frcJumpstart.newMechanism',     cmdMechanism),
        vscode.commands.registerCommand('frcJumpstart.newClimb',         cmdClimb),
        vscode.commands.registerCommand('frcJumpstart.newBuddyClimb',    cmdBuddyClimb),
        vscode.commands.registerCommand('frcJumpstart.newClaw',          cmdClaw),
        vscode.commands.registerCommand('frcJumpstart.newPneumatics',    cmdPneumatics),
        vscode.commands.registerCommand('frcJumpstart.newVision',        cmdVision),
        vscode.commands.registerCommand('frcJumpstart.newLeds',          cmdLeds),
        vscode.commands.registerCommand('frcJumpstart.newPathPlanner',   cmdPathPlanner),
        vscode.commands.registerCommand('frcJumpstart.newChoreo',        cmdChoreo),
        vscode.commands.registerCommand('frcJumpstart.combine',          cmdCombine),
        vscode.window.registerWebviewViewProvider(FrcForgeViewProvider.viewId, new FrcForgeViewProvider()),
    );
}

export function deactivate() {}
