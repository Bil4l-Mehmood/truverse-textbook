/**
 * Sign-up form component with background questionnaire
 * Uses Better Auth for 50 bonus competition points
 */

import React, { useState } from 'react';
import { useAuthStore } from '../../store/authStore';
import { betterAuthSignUp } from '../../services/betterAuthService';

export default function SignUpForm() {
  const setAuth = useAuthStore((state) => state.setAuth);

  const [step, setStep] = useState<'basic' | 'background'>(  'basic');
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  // Basic info
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [name, setName] = useState('');

  // Background questionnaire
  const [ros2Experience, setRos2Experience] = useState('Beginner');
  const [gpuModel, setGpuModel] = useState('');
  const [gpuVram, setGpuVram] = useState('');
  const [operatingSystem, setOperatingSystem] = useState('');
  const [roboticsKnowledge, setRoboticsKnowledge] = useState('Beginner');

  const handleBasicSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);

    // Validation
    if (!email || !password || !name) {
      setError('All fields are required');
      return;
    }

    if (password.length < 8) {
      setError('Password must be at least 8 characters');
      return;
    }

    // Move to background questionnaire
    setStep('background');
  };

  const handleBackgroundSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);
    setLoading(true);

    console.log('[SignUp] Starting account creation...', {
      email,
      name,
      ros2Experience,
      operatingSystem,
      roboticsKnowledge,
    });

    try {
      console.log('[SignUp] Calling Better Auth signup API...');
      const response = await betterAuthSignUp({
        email,
        password,
        name,
        ros2_experience: ros2Experience,
        gpu_model: gpuModel || undefined,
        gpu_vram: gpuVram || undefined,
        operating_system: operatingSystem || undefined,
        robotics_knowledge: roboticsKnowledge,
      });

      console.log('[SignUp] Signup successful with Better Auth:', response.user);

      // Store auth state (convert Better Auth session to our format)
      const authUser = {
        id: response.user.id, // UUID string, don't parseInt
        email: response.user.email,
        name: response.user.name,
        ros2_experience: response.user.ros2_experience,
        gpu_model: response.user.gpu_model,
        gpu_vram: response.user.gpu_vram,
        operating_system: response.user.operating_system,
        robotics_knowledge: response.user.robotics_knowledge,
      };
      setAuth(authUser, response.session.token);

      console.log('[SignUp] Auth state stored, redirecting to home...');

      // Redirect to home
      window.location.href = '/';
    } catch (err) {
      console.error('[SignUp] Better Auth signup failed:', err);
      const errorMessage = err instanceof Error ? err.message : 'Sign-up failed. Please try again.';
      setError(errorMessage);
    } finally {
      setLoading(false);
    }
  };

  const handleBack = () => {
    // Go back to step 1
    setPassword('');
    setError(null);
    setStep('basic');
  };

  const handleSkipQuestionnaire = async () => {
    setError(null);
    setLoading(true);

    console.log('[SignUp] Skipping questionnaire, creating account with defaults...');

    try {
      const response = await betterAuthSignUp({
        email,
        password,
        name,
      });

      console.log('[SignUp] Signup successful (skipped) with Better Auth:', response.user);

      // Store auth state (convert Better Auth session to our format)
      const authUser = {
        id: response.user.id, // UUID string, don't parseInt
        email: response.user.email,
        name: response.user.name,
        ros2_experience: response.user.ros2_experience,
        gpu_model: response.user.gpu_model,
        gpu_vram: response.user.gpu_vram,
        operating_system: response.user.operating_system,
        robotics_knowledge: response.user.robotics_knowledge,
      };
      setAuth(authUser, response.session.token);

      console.log('[SignUp] Redirecting to home...');
      window.location.href = '/';
    } catch (err) {
      console.error('[SignUp] Better Auth signup failed:', err);
      const errorMessage = err instanceof Error ? err.message : 'Sign-up failed. Please try again.';
      setError(errorMessage);
    } finally {
      setLoading(false);
    }
  };

  if (step === 'basic') {
    return (
      <div className="auth-container">
        <div className="auth-card">
          <h1>Sign Up</h1>
          <p className="auth-subtitle">Create your account to get started</p>

          {error && <div className="auth-error">{error}</div>}

          <form onSubmit={handleBasicSubmit} className="auth-form">
            <div className="form-group">
              <label htmlFor="name">Full Name</label>
              <input
                type="text"
                id="name"
                value={name}
                onChange={(e) => setName(e.target.value)}
                placeholder="John Doe"
                required
              />
            </div>

            <div className="form-group">
              <label htmlFor="email">Email</label>
              <input
                type="email"
                id="email"
                value={email}
                onChange={(e) => setEmail(e.target.value)}
                placeholder="you@example.com"
                required
              />
            </div>

            <div className="form-group">
              <label htmlFor="password">Password</label>
              <input
                type="password"
                id="password"
                value={password}
                onChange={(e) => setPassword(e.target.value)}
                placeholder="At least 8 characters"
                minLength={8}
                required
              />
            </div>

            <button type="submit" className="btn-primary">
              Continue to Background Questionnaire
            </button>
          </form>

          <p className="auth-footer">
            Already have an account? <a href="/signin">Sign in</a>
          </p>
        </div>
      </div>
    );
  }

  return (
    <div className="auth-container">
      <div className="auth-card">
        <h1>Background Questionnaire</h1>
        <p className="auth-subtitle">
          Help us personalize your learning experience
        </p>

        {error && <div className="auth-error">{error}</div>}

        <form onSubmit={handleBackgroundSubmit} className="auth-form">
          <div className="form-group">
            <label htmlFor="ros2-experience">ROS 2 Experience Level</label>
            <select
              id="ros2-experience"
              value={ros2Experience}
              onChange={(e) => setRos2Experience(e.target.value)}
            >
              <option value="None">None</option>
              <option value="Beginner">Beginner</option>
              <option value="Intermediate">Intermediate</option>
              <option value="Advanced">Advanced</option>
            </select>
          </div>

          <div className="form-group">
            <label htmlFor="gpu-model">GPU Model (Optional)</label>
            <input
              type="text"
              id="gpu-model"
              value={gpuModel}
              onChange={(e) => setGpuModel(e.target.value)}
              placeholder="e.g., NVIDIA RTX 3060"
            />
          </div>

          <div className="form-group">
            <label htmlFor="gpu-vram">GPU VRAM (Optional)</label>
            <input
              type="text"
              id="gpu-vram"
              value={gpuVram}
              onChange={(e) => setGpuVram(e.target.value)}
              placeholder="e.g., 12GB"
            />
          </div>

          <div className="form-group">
            <label htmlFor="os">Operating System (Optional)</label>
            <select
              id="os"
              value={operatingSystem}
              onChange={(e) => setOperatingSystem(e.target.value)}
            >
              <option value="">Select...</option>
              <option value="Ubuntu">Ubuntu</option>
              <option value="Windows">Windows</option>
              <option value="macOS">macOS</option>
            </select>
          </div>

          <div className="form-group">
            <label htmlFor="robotics-knowledge">Robotics Knowledge Level</label>
            <select
              id="robotics-knowledge"
              value={roboticsKnowledge}
              onChange={(e) => setRoboticsKnowledge(e.target.value)}
            >
              <option value="None">None</option>
              <option value="Beginner">Beginner</option>
              <option value="Intermediate">Intermediate</option>
              <option value="Advanced">Advanced</option>
            </select>
          </div>

          <div className="form-actions" style={{ display: 'flex', gap: '0.5rem', justifyContent: 'space-between' }}>
            <button
              type="button"
              onClick={handleBack}
              className="btn-secondary"
              disabled={loading}
            >
              ← Back
            </button>
            <div style={{ display: 'flex', gap: '0.5rem' }}>
              <button
                type="button"
                onClick={handleSkipQuestionnaire}
                className="btn-secondary"
                disabled={loading}
              >
                Skip for Now
              </button>
              <button type="submit" className="btn-primary" disabled={loading}>
                {loading ? 'Creating Account...' : 'Complete Sign Up'}
              </button>
            </div>
          </div>
        </form>
      </div>
    </div>
  );
}
