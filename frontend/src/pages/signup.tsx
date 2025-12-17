/**
 * Sign-up page
 */

import React from 'react';
import Layout from '@theme/Layout';
import SignUpForm from '../components/Auth/SignUpForm';

export default function SignUpPage() {
  return (
    <Layout title="Sign Up" description="Create your account">
      <SignUpForm />
    </Layout>
  );
}
