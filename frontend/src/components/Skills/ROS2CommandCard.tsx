import React, { useState } from 'react';
import { ROS2Command } from '../../services/skillsService';
import styles from './SkillsCards.module.css';

interface ROS2CommandCardProps {
  command: ROS2Command;
}

export default function ROS2CommandCard({ command }: ROS2CommandCardProps) {
  const [copied, setCopied] = useState(false);

  const copyToClipboard = () => {
    if (command.command) {
      navigator.clipboard.writeText(command.command);
      setCopied(true);
      setTimeout(() => setCopied(false), 2000);
    }
  };

  if (command.error) {
    return (
      <div className={styles.errorCard}>
        <div className={styles.errorIcon}>⚠️</div>
        <p className={styles.errorMessage}>{command.error}</p>
      </div>
    );
  }

  if (command.clarification_needed) {
    return (
      <div className={styles.clarificationCard}>
        <div className={styles.clarificationIcon}>❓</div>
        <p className={styles.clarificationText}>{command.clarification_needed}</p>
        {command.suggestions && (
          <div className={styles.suggestions}>
            <p>Please clarify:</p>
            <ul>
              {command.suggestions.map((suggestion, index) => (
                <li key={index}>{suggestion}</li>
              ))}
            </ul>
          </div>
        )}
      </div>
    );
  }

  return (
    <div className={styles.ros2Card}>
      <div className={styles.cardHeader}>
        <span className={styles.icon}>🤖</span>
        <h3>ROS 2 Command</h3>
      </div>

      <div className={styles.taskDescription}>
        <label>Task</label>
        <p>{command.task}</p>
      </div>

      <div className={styles.commandBlock}>
        <div className={styles.commandHeader}>
          <label>Command</label>
          <button className={styles.copyBtn} onClick={copyToClipboard}>
            {copied ? '✓ Copied!' : '📋 Copy'}
          </button>
        </div>
        <pre className={styles.commandCode}>{command.command}</pre>
      </div>

      <div className={styles.explanation}>
        <label>What this does</label>
        <p>{command.explanation}</p>
      </div>

      {command.parameters && command.parameters.length > 0 && (
        <div className={styles.parameters}>
          <label>Parameters</label>
          <table className={styles.parametersTable}>
            <thead>
              <tr>
                <th>Parameter</th>
                <th>Description</th>
                {command.parameters.some(p => p.default) && <th>Default</th>}
              </tr>
            </thead>
            <tbody>
              {command.parameters.map((param, index) => (
                <tr key={index}>
                  <td className={styles.paramName}>{param.name}</td>
                  <td>{param.description}</td>
                  {command.parameters.some(p => p.default) && (
                    <td className={styles.paramDefault}>{param.default || '-'}</td>
                  )}
                </tr>
              ))}
            </tbody>
          </table>
        </div>
      )}
    </div>
  );
}
