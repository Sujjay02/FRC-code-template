import * as vscode from 'vscode';
import * as fs from 'fs';
import * as path from 'path';

export interface ProjectInfo {
    year: string;
    language: string;
}

export function getProjectInfo(): ProjectInfo | null {
    const folders = vscode.workspace.workspaceFolders;
    if (!folders?.length) { return null; }
    const prefsPath = path.join(folders[0].uri.fsPath, '.wpilib', 'wpilib_preferences.json');
    try {
        const raw = fs.readFileSync(prefsPath, 'utf8');
        const prefs = JSON.parse(raw);
        return {
            year:     String(prefs.projectYear     ?? '?'),
            language: String(prefs.currentLanguage ?? '?'),
        };
    } catch { return null; }
}

export function langDisplay(lang: string): string {
    switch (lang.toLowerCase()) {
        case 'java':   return 'Java';
        case 'cpp':    return 'C++';
        case 'kotlin': return 'Kotlin';
        default:       return lang;
    }
}
