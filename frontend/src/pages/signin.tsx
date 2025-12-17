/**
 * Sign-in page
 */

import React from 'react';
import Layout from '@theme/Layout';
import SignInForm from '../components/Auth/SignInForm';

export default function SignInPage() {
  return (
    <Layout title="Sign In" description="Sign in to your account">
      <SignInForm />
    </Layout>
  );
}
